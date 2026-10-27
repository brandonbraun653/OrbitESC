#include <gtest/gtest.h>

#include <cstring>
#include <ostream>

#include "protocol/MessageCodec.h"
#include "protocol/ParameterCodec.h"

using namespace orbit::protocol;

TEST( MessageCodec, PingRoundTrip )
{
  const QByteArray payload = encodePing( 42 );
  ASSERT_FALSE( payload.isEmpty() );

  const auto header = peekHeader( payload );
  ASSERT_TRUE( header.has_value() );
  EXPECT_EQ( header->msgId, static_cast<quint8>( MsgId_MSG_PING_CMD ) );
  EXPECT_EQ( header->uuid, static_cast<quint16>( 42 ) );

  const auto message = decodeMessage<PingMessage>( payload, PingMessage_fields );
  EXPECT_TRUE( message.has_value() );
}

TEST( MessageCodec, SystemControlHeader )
{
  const QByteArray payload = encodeSystemControl( SystemControlSubId_FAULT, 7 );
  const auto header        = peekHeader( payload );
  ASSERT_TRUE( header.has_value() );
  EXPECT_EQ( header->msgId, static_cast<quint8>( MsgId_MSG_SYS_CTRL ) );
  EXPECT_EQ( header->subId, static_cast<quint8>( SystemControlSubId_FAULT ) );
  EXPECT_EQ( header->uuid, static_cast<quint16>( 7 ) );
}

TEST( MessageCodec, ParameterGet )
{
  const QByteArray payload = encodeParamGet( ParamId_PARAM_BOOT_COUNT, 11 );
  const auto message       = decodeMessage<ParamIOMessage>( payload, ParamIOMessage_fields );
  ASSERT_TRUE( message.has_value() );
  EXPECT_TRUE( message->has_id );
  EXPECT_EQ( message->id, ParamId_PARAM_BOOT_COUNT );
  EXPECT_EQ( message->header.subId, static_cast<quint8>( ParamIOSubId_GET ) );
}

TEST( MessageCodec, ParameterSet )
{
  bool ok                = false;
  const QByteArray data  = encodeParameterValue( ParamType_FLOAT, 12.5f, &ok );
  ASSERT_TRUE( ok );

  const QByteArray payload = encodeParamSet( ParamId_PARAM_STATOR_PWM_FREQ, ParamType_FLOAT, data, 12 );
  const auto message       = decodeMessage<ParamIOMessage>( payload, ParamIOMessage_fields );
  ASSERT_TRUE( message.has_value() );
  EXPECT_TRUE( message->has_type );
  EXPECT_TRUE( message->has_data );
  EXPECT_EQ( message->type, ParamType_FLOAT );
  EXPECT_EQ( static_cast<int>( message->data.size ), 4 );
}

struct ParameterValueCase
{
  const char *name;
  ParamType type;
  QVariant value;
};

void PrintTo( const ParameterValueCase &testCase, std::ostream *stream )
{
  *stream << testCase.name;
}

class ParameterValueRoundTrip : public ::testing::TestWithParam<ParameterValueCase>
{
};

TEST_P( ParameterValueRoundTrip, EncodesAndDecodes )
{
  const ParameterValueCase &testCase = GetParam();

  bool encoded           = false;
  const QByteArray data  = encodeParameterValue( testCase.type, testCase.value, &encoded );
  ASSERT_TRUE( encoded );

  bool decoded        = false;
  const QVariant result = decodeParameterValue( testCase.type, data, &decoded );
  ASSERT_TRUE( decoded );

  switch( testCase.type )
  {
    case ParamType_BOOL:
      EXPECT_EQ( result.toBool(), testCase.value.toBool() );
      break;
    case ParamType_UINT8:
    case ParamType_UINT16:
    case ParamType_UINT32:
      EXPECT_EQ( result.toULongLong(), testCase.value.toULongLong() );
      break;
    case ParamType_FLOAT:
      EXPECT_FLOAT_EQ( result.toFloat(), testCase.value.toFloat() );
      break;
    case ParamType_DOUBLE:
      EXPECT_DOUBLE_EQ( result.toDouble(), testCase.value.toDouble() );
      break;
    case ParamType_STRING:
      EXPECT_EQ( result.toString(), testCase.value.toString() );
      break;
    case ParamType_BYTES:
      EXPECT_EQ( result.toByteArray(), testCase.value.toByteArray() );
      break;
    default:
      FAIL() << "Unexpected parameter type";
  }
}

INSTANTIATE_TEST_SUITE_P( MessageCodec, ParameterValueRoundTrip,
                          ::testing::Values( ParameterValueCase{ "bool", ParamType_BOOL, QVariant( true ) },
                                             ParameterValueCase{ "uint8", ParamType_UINT8, QVariant( 200 ) },
                                             ParameterValueCase{ "uint16", ParamType_UINT16, QVariant( 50000 ) },
                                             ParameterValueCase{ "uint32", ParamType_UINT32,
                                                                 QVariant( 3000000000LL ) },
                                             ParameterValueCase{ "float", ParamType_FLOAT, QVariant( 1.25 ) },
                                             ParameterValueCase{ "double", ParamType_DOUBLE,
                                                                 QVariant( 3.1415926535 ) },
                                             ParameterValueCase{ "string", ParamType_STRING,
                                                                 QVariant( "OrbitESC" ) },
                                             ParameterValueCase{ "bytes", ParamType_BYTES,
                                                                 QVariant( QByteArray( "\x01\x02", 2 ) ) } ),
                          testing::PrintToStringParamName() );

TEST( MessageCodec, SystemVoltages )
{
  ADCSystemVoltagesPayload voltages{};
  voltages.v_mcu     = 3.3f;
  voltages.v_dc_link = 12.0f;
  voltages.v_temp    = 1.7f;
  voltages.v_isense  = 1.65f;

  const QByteArray payload = encodeMessage( voltages, ADCSystemVoltagesPayload_fields );
  ASSERT_FALSE( payload.isEmpty() );

  SystemDataMessage message = SystemDataMessage_init_zero;
  message.id                = SystemDataId_ADC_SYSTEM_VOLTAGES;
  message.has_payload       = true;
  message.payload.size      = static_cast<pb_size_t>( payload.size() );
  std::memcpy( message.payload.bytes, payload.constData(), static_cast<size_t>( payload.size() ) );

  const auto decoded = decodeSystemVoltages( message );
  ASSERT_TRUE( decoded.has_value() );
  EXPECT_FLOAT_EQ( decoded->vDcLink, 12.0f );
  EXPECT_FLOAT_EQ( decoded->vTemp, 1.7f );
}

TEST( MessageCodec, PhaseCurrents )
{
  ADCPhaseCurrentsPayload currents{};
  currents.ia = 1.0f;
  currents.ib = -2.0f;
  currents.ic = 0.5f;

  const QByteArray payload = encodeMessage( currents, ADCPhaseCurrentsPayload_fields );
  ASSERT_FALSE( payload.isEmpty() );

  SystemDataMessage message = SystemDataMessage_init_zero;
  message.id                = SystemDataId_ADC_PHASE_CURRENTS;
  message.has_payload       = true;
  message.payload.size      = static_cast<pb_size_t>( payload.size() );
  std::memcpy( message.payload.bytes, payload.constData(), static_cast<size_t>( payload.size() ) );

  const auto decoded = decodePhaseCurrents( message );
  ASSERT_TRUE( decoded.has_value() );
  EXPECT_FLOAT_EQ( decoded->ia, 1.0f );
  EXPECT_FLOAT_EQ( decoded->ib, -2.0f );
  EXPECT_FLOAT_EQ( decoded->ic, 0.5f );
}

TEST( MessageCodec, InvalidHeaderIsRejected )
{
  EXPECT_FALSE( peekHeader( QByteArray::fromHex( "deadbeef" ) ).has_value() );
}
