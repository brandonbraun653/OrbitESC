#include <QtTest>

#include "protocol/MessageCodec.h"
#include "protocol/ParameterCodec.h"

using namespace orbit::protocol;

class TestMessageCodec : public QObject
{
  Q_OBJECT

private slots:
  void pingRoundTrip()
  {
    const QByteArray payload = encodePing( 42 );
    QVERIFY( !payload.isEmpty() );

    const auto header = peekHeader( payload );
    QVERIFY( header.has_value() );
    QCOMPARE( header->msgId, static_cast<quint8>( MsgId_MSG_PING_CMD ) );
    QCOMPARE( header->uuid, static_cast<quint16>( 42 ) );

    const auto message = decodeMessage<PingMessage>( payload, PingMessage_fields );
    QVERIFY( message.has_value() );
  }

  void systemControlHeader()
  {
    const QByteArray payload = encodeSystemControl( SystemControlSubId_FAULT, 7 );
    const auto header = peekHeader( payload );
    QVERIFY( header.has_value() );
    QCOMPARE( header->msgId, static_cast<quint8>( MsgId_MSG_SYS_CTRL ) );
    QCOMPARE( header->subId, static_cast<quint8>( SystemControlSubId_FAULT ) );
    QCOMPARE( header->uuid, static_cast<quint16>( 7 ) );
  }

  void parameterGet()
  {
    const QByteArray payload = encodeParamGet( ParamId_PARAM_BOOT_COUNT, 11 );
    const auto message = decodeMessage<ParamIOMessage>( payload, ParamIOMessage_fields );
    QVERIFY( message.has_value() );
    QVERIFY( message->has_id );
    QCOMPARE( message->id, ParamId_PARAM_BOOT_COUNT );
    QCOMPARE( message->header.subId, static_cast<quint8>( ParamIOSubId_GET ) );
  }

  void parameterSet()
  {
    bool ok = false;
    const QByteArray data = encodeParameterValue( ParamType_FLOAT, 12.5f, &ok );
    QVERIFY( ok );

    const QByteArray payload = encodeParamSet( ParamId_PARAM_STATOR_PWM_FREQ, ParamType_FLOAT, data, 12 );
    const auto message = decodeMessage<ParamIOMessage>( payload, ParamIOMessage_fields );
    QVERIFY( message.has_value() );
    QVERIFY( message->has_type );
    QVERIFY( message->has_data );
    QCOMPARE( message->type, ParamType_FLOAT );
    QCOMPARE( static_cast<int>( message->data.size ), 4 );
  }

  void parameterValueRoundTrips_data()
  {
    QTest::addColumn<int>( "type" );
    QTest::addColumn<QVariant>( "value" );

    QTest::newRow( "bool" ) << static_cast<int>( ParamType_BOOL ) << QVariant( true );
    QTest::newRow( "uint8" ) << static_cast<int>( ParamType_UINT8 ) << QVariant( 200 );
    QTest::newRow( "uint16" ) << static_cast<int>( ParamType_UINT16 ) << QVariant( 50000 );
    QTest::newRow( "uint32" ) << static_cast<int>( ParamType_UINT32 ) << QVariant( 3000000000LL );
    QTest::newRow( "float" ) << static_cast<int>( ParamType_FLOAT ) << QVariant( 1.25 );
    QTest::newRow( "double" ) << static_cast<int>( ParamType_DOUBLE ) << QVariant( 3.1415926535 );
    QTest::newRow( "string" ) << static_cast<int>( ParamType_STRING ) << QVariant( "OrbitESC" );
    QTest::newRow( "bytes" ) << static_cast<int>( ParamType_BYTES ) << QVariant( QByteArray( "\x01\x02", 2 ) );
  }

  void parameterValueRoundTrips()
  {
    QFETCH( int, type );
    QFETCH( QVariant, value );

    bool encoded = false;
    const QByteArray data = encodeParameterValue( static_cast<ParamType>( type ), value, &encoded );
    QVERIFY( encoded );

    bool decoded = false;
    const QVariant result = decodeParameterValue( static_cast<ParamType>( type ), data, &decoded );
    QVERIFY( decoded );

    if( type == static_cast<int>( ParamType_FLOAT ) )
    {
      QCOMPARE( result.toFloat(), value.toFloat() );
    }
    else if( type == static_cast<int>( ParamType_DOUBLE ) )
    {
      QCOMPARE( result.toDouble(), value.toDouble() );
    }
    else
    {
      QCOMPARE( result, value );
    }
  }

  void systemVoltages()
  {
    ADCSystemVoltagesPayload voltages{};
    voltages.v_mcu     = 3.3f;
    voltages.v_dc_link = 12.0f;
    voltages.v_temp    = 1.7f;
    voltages.v_isense  = 1.65f;

    const QByteArray payload = encodeMessage( voltages, ADCSystemVoltagesPayload_fields );
    QVERIFY( !payload.isEmpty() );

    SystemDataMessage message = SystemDataMessage_init_zero;
    message.id = SystemDataId_ADC_SYSTEM_VOLTAGES;
    message.has_payload = true;
    message.payload.size = static_cast<pb_size_t>( payload.size() );
    memcpy( message.payload.bytes, payload.constData(), static_cast<size_t>( payload.size() ) );

    const auto decoded = decodeSystemVoltages( message );
    QVERIFY( decoded.has_value() );
    QCOMPARE( decoded->vDcLink, 12.0f );
    QCOMPARE( decoded->vTemp, 1.7f );
  }

  void phaseCurrents()
  {
    ADCPhaseCurrentsPayload currents{};
    currents.ia = 1.0f;
    currents.ib = -2.0f;
    currents.ic = 0.5f;

    const QByteArray payload = encodeMessage( currents, ADCPhaseCurrentsPayload_fields );
    QVERIFY( !payload.isEmpty() );

    SystemDataMessage message = SystemDataMessage_init_zero;
    message.id = SystemDataId_ADC_PHASE_CURRENTS;
    message.has_payload = true;
    message.payload.size = static_cast<pb_size_t>( payload.size() );
    memcpy( message.payload.bytes, payload.constData(), static_cast<size_t>( payload.size() ) );

    const auto decoded = decodePhaseCurrents( message );
    QVERIFY( decoded.has_value() );
    QCOMPARE( decoded->ia, 1.0f );
    QCOMPARE( decoded->ib, -2.0f );
    QCOMPARE( decoded->ic, 0.5f );
  }

  void invalidHeaderIsRejected()
  {
    QVERIFY( !peekHeader( QByteArray::fromHex( "deadbeef" ) ).has_value() );
  }
};

QTEST_GUILESS_MAIN( TestMessageCodec )
#include "tst_message_codec.moc"
