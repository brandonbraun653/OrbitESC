#include <QtEndian>
#include <QtTest>

#include "device/DeviceClient.h"
#include "protocol/FrameCodec.h"
#include "protocol/MessageCodec.h"
#include "protocol/ParameterCodec.h"

using namespace orbit::device;
using namespace orbit::protocol;

namespace
{
  class FakeTransport : public Transport
  {
  public:
    std::function<void( const QByteArray & )> onWrite;
    QList<QByteArray> writes;

    bool open( const QString & ) override
    {
      QTimer::singleShot( 0, this, [this]() { emit opened(); } );
      return true;
    }

    void close() override
    {
      QTimer::singleShot( 0, this, [this]() { emit closed(); } );
    }

    void write( const QByteArray &data ) override
    {
      writes.append( data );
      if( onWrite )
      {
        QTimer::singleShot( 0, this, [this, data]() { onWrite( data ); } );
      }
    }

    bool isOpen() const override
    {
      return true;
    }

    void inject( const QByteArray &payload )
    {
      emit dataReceived( encodeFrame( payload ) );
    }

    std::optional<HeaderView> lastHeader() const
    {
      if( writes.isEmpty() )
      {
        return std::nullopt;
      }

      FrameAccumulator accumulator;
      accumulator.append( writes.last() );
      const auto frames = accumulator.takeFrames();
      return frames.payloads.isEmpty() ? std::nullopt : peekHeader( frames.payloads.first() );
    }
  };

  QByteArray ackFor( const HeaderView &header )
  {
    AckNackMessage ack = AckNackMessage_init_zero;
    ack.header.msgId = MsgId_MSG_ACK_NACK;
    ack.header.subId = header.subId;
    ack.header.uuid  = header.uuid;
    ack.acknowledge  = true;
    ack.status_code  = StatusCode_NO_ERROR;
    return encodeMessage( ack, AckNackMessage_fields );
  }

  QByteArray tickMessage( quint32 tick )
  {
    SystemTickMessage message = SystemTickMessage_init_zero;
    message.header.msgId = MsgId_MSG_SYS_TICK;
    message.tick         = tick;
    return encodeMessage( message, SystemTickMessage_fields );
  }
}    // namespace

class TestDeviceClient : public QObject
{
  Q_OBJECT

private slots:
  void connectPingAndTick()
  {
    FakeTransport transport;
    DeviceClient client( &transport );
    QSignalSpy stateSpy( &client, &DeviceClient::stateChanged );

    transport.onWrite = [&transport]( const QByteArray &frame )
    {
      Q_UNUSED( frame )
      const auto header = transport.lastHeader();
      if( header.has_value() && header->msgId == MsgId_MSG_PING_CMD )
      {
        transport.inject( ackFor( *header ) );
      }
    };

    client.openPort( "fake" );
    QTRY_VERIFY( stateSpy.count() >= 1 );
    QCOMPARE( client.state(), DeviceClient::State::Connecting );
    QTRY_VERIFY( transport.writes.size() >= 1 );

    transport.inject( tickMessage( 1 ) );
    QTRY_COMPARE( client.state(), DeviceClient::State::Online );
  }

  void parameterResponseIsCorrelated()
  {
    FakeTransport transport;
    DeviceClient client( &transport );

    transport.onWrite = [&transport]( const QByteArray & )
    {
      const auto header = transport.lastHeader();
      if( !header.has_value() )
      {
        return;
      }

      if( header->msgId == MsgId_MSG_PING_CMD )
      {
        transport.inject( ackFor( *header ) );
      }
      else if( header->msgId == MsgId_MSG_PARAM_IO && header->subId == ParamIOSubId_GET )
      {
        ParamIOMessage reply = ParamIOMessage_init_zero;
        reply.header.msgId = MsgId_MSG_PARAM_IO;
        reply.header.subId = ParamIOSubId_GET;
        reply.header.uuid  = header->uuid;
        reply.has_type     = true;
        reply.type         = ParamType_UINT32;
        reply.has_data     = true;
        reply.data.size    = 4;
        qToLittleEndian<quint32>( 9, reply.data.bytes );
        transport.inject( encodeMessage( reply, ParamIOMessage_fields ) );
      }
    };

    client.openPort( "fake" );
    QTRY_VERIFY( transport.writes.size() >= 1 );
    transport.inject( tickMessage( 1 ) );
    QTRY_COMPARE( client.state(), DeviceClient::State::Online );

    QSignalSpy valueSpy( &client, &DeviceClient::parameterValueReceived );
    QSignalSpy doneSpy( &client, &DeviceClient::commandFinished );
    client.requestParameterGet( ParamId_PARAM_BOOT_COUNT );

    QTRY_VERIFY( valueSpy.count() >= 1 );
    QCOMPARE( valueSpy.first().at( 0 ).toInt(), static_cast<int>( ParamId_PARAM_BOOT_COUNT ) );
    QVERIFY( doneSpy.count() >= 1 );
  }

  void streamCommandUsesCorrectSubId()
  {
    FakeTransport transport;
    DeviceClient client( &transport );

    transport.onWrite = [&transport]( const QByteArray & )
    {
      const auto header = transport.lastHeader();
      if( header.has_value() )
      {
        transport.inject( ackFor( *header ) );
      }
    };

    client.openPort( "fake" );
    QTRY_VERIFY( transport.writes.size() >= 1 );
    transport.inject( tickMessage( 1 ) );
    QTRY_COMPARE( client.state(), DeviceClient::State::Online );

    QSignalSpy doneSpy( &client, &DeviceClient::commandFinished );
    client.setPhaseCurrentStream( true );

    QTRY_VERIFY( doneSpy.count() >= 1 );
    const auto header = transport.lastHeader();
    QVERIFY( header.has_value() );
    QCOMPARE( header->msgId, static_cast<quint8>( MsgId_MSG_SYS_CTRL ) );
    QCOMPARE( header->subId, static_cast<quint8>( SystemControlSubId_ENABLE_STREAM_PHASE_CURRENTS ) );
  }

  void parameterWriteIsAcknowledged()
  {
    FakeTransport transport;
    DeviceClient client( &transport );

    transport.onWrite = [&transport]( const QByteArray & )
    {
      const auto header = transport.lastHeader();
      if( header.has_value() )
      {
        transport.inject( ackFor( *header ) );
      }
    };

    client.openPort( "fake" );
    QTRY_VERIFY( transport.writes.size() >= 1 );
    transport.inject( tickMessage( 1 ) );
    QTRY_COMPARE( client.state(), DeviceClient::State::Online );

    bool encoded = false;
    const QByteArray data = encodeParameterValue( ParamType_FLOAT, 1.25f, &encoded );
    QVERIFY( encoded );

    QSignalSpy writeSpy( &client, &DeviceClient::parameterWriteFinished );
    client.requestParameterSet( ParamId_PARAM_ACTIVITY_LED_SCALER, ParamType_FLOAT, data );

    QTRY_VERIFY( writeSpy.count() >= 1 );
    QCOMPARE( writeSpy.first().at( 0 ).toInt(), static_cast<int>( ParamId_PARAM_ACTIVITY_LED_SCALER ) );
    QCOMPARE( writeSpy.first().at( 1 ).toBool(), true );

    const auto header = transport.lastHeader();
    QVERIFY( header.has_value() );
    QCOMPARE( header->msgId, static_cast<quint8>( MsgId_MSG_PARAM_IO ) );
    QCOMPARE( header->subId, static_cast<quint8>( ParamIOSubId_SET ) );
  }

  void tickTimeoutGoesOffline()
  {
    FakeTransport transport;
    DeviceClient client( &transport );

    transport.onWrite = [&transport]( const QByteArray & )
    {
      const auto header = transport.lastHeader();
      if( header.has_value() && header->msgId == MsgId_MSG_PING_CMD )
      {
        transport.inject( ackFor( *header ) );
      }
    };

    client.openPort( "fake" );
    QTRY_VERIFY( transport.writes.size() >= 1 );
    transport.inject( tickMessage( 1 ) );
    QTRY_COMPARE( client.state(), DeviceClient::State::Online );

    QTRY_COMPARE_WITH_TIMEOUT( client.state(), DeviceClient::State::Offline, 5000 );
  }

  void commandsAreRejectedWhenOffline()
  {
    FakeTransport transport;
    DeviceClient client( &transport );
    QSignalSpy doneSpy( &client, &DeviceClient::commandFinished );

    client.requestParameterGet( ParamId_PARAM_BOOT_COUNT );
    QCOMPARE( doneSpy.count(), 1 );
    QCOMPARE( doneSpy.first().at( 2 ).toBool(), false );
  }
};

QTEST_GUILESS_MAIN( TestDeviceClient )
#include "tst_device_client.moc"
