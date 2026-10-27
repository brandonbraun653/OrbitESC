#include "support/MessageFactory.h"

#include <cstring>

namespace orbit::test
{
  using namespace orbit::protocol;

  QByteArray frame( const QByteArray &payload )
  {
    return encodeFrame( payload );
  }

  QByteArray ackFor( const HeaderView &header )
  {
    AckNackMessage ack = AckNackMessage_init_zero;
    ack.header.msgId   = MsgId_MSG_ACK_NACK;
    ack.header.subId   = header.subId;
    ack.header.uuid    = header.uuid;
    ack.acknowledge    = true;
    ack.status_code    = StatusCode_NO_ERROR;
    return encodeMessage( ack, AckNackMessage_fields );
  }

  QByteArray nackFor( const HeaderView &header, StatusCode status )
  {
    AckNackMessage nack = AckNackMessage_init_zero;
    nack.header.msgId   = MsgId_MSG_ACK_NACK;
    nack.header.subId   = header.subId;
    nack.header.uuid    = header.uuid;
    nack.acknowledge    = false;
    nack.status_code    = status;
    return encodeMessage( nack, AckNackMessage_fields );
  }

  QByteArray tickMessage( quint32 tick )
  {
    SystemTickMessage message = SystemTickMessage_init_zero;
    message.header.msgId      = MsgId_MSG_SYS_TICK;
    message.tick              = tick;
    return encodeMessage( message, SystemTickMessage_fields );
  }

  QByteArray statusMessage( MotorCtrlState state, quint32 tick )
  {
    SystemStatusMessage message = SystemStatusMessage_init_zero;
    message.header.msgId        = MsgId_MSG_SYS_STATUS;
    message.systemTick          = tick;
    message.motorCtrlState      = state;
    return encodeMessage( message, SystemStatusMessage_fields );
  }

  QByteArray voltagesMessage( float vMcu, float vDcLink, float vTemp, float vIsense )
  {
    ADCSystemVoltagesPayload voltages{};
    voltages.v_mcu     = vMcu;
    voltages.v_dc_link = vDcLink;
    voltages.v_temp    = vTemp;
    voltages.v_isense  = vIsense;

    const QByteArray payload = encodeMessage( voltages, ADCSystemVoltagesPayload_fields );
    if( payload.isEmpty() )
    {
      return {};
    }

    SystemDataMessage message = SystemDataMessage_init_zero;
    message.header.msgId      = MsgId_MSG_SYS_DATA;
    message.id                = SystemDataId_ADC_SYSTEM_VOLTAGES;
    message.has_payload       = true;
    message.payload.size      = static_cast<pb_size_t>( payload.size() );
    std::memcpy( message.payload.bytes, payload.constData(), static_cast<size_t>( payload.size() ) );
    return encodeMessage( message, SystemDataMessage_fields );
  }

  QByteArray phaseCurrentsMessage( float ia, float ib, float ic, quint32 timestampUs )
  {
    ADCPhaseCurrentsPayload currents{};
    currents.ia = ia;
    currents.ib = ib;
    currents.ic = ic;

    const QByteArray payload = encodeMessage( currents, ADCPhaseCurrentsPayload_fields );
    if( payload.isEmpty() )
    {
      return {};
    }

    SystemDataMessage message = SystemDataMessage_init_zero;
    message.header.msgId      = MsgId_MSG_SYS_DATA;
    message.id                = SystemDataId_ADC_PHASE_CURRENTS;
    message.timestamp         = timestampUs;
    message.has_payload       = true;
    message.payload.size      = static_cast<pb_size_t>( payload.size() );
    std::memcpy( message.payload.bytes, payload.constData(), static_cast<size_t>( payload.size() ) );
    return encodeMessage( message, SystemDataMessage_fields );
  }

  QByteArray parameterReply( quint16 uuid, ParamId paramId, ParamType type, const QByteArray &data )
  {
    ParamIOMessage reply = ParamIOMessage_init_zero;
    reply.header.msgId   = MsgId_MSG_PARAM_IO;
    reply.header.subId   = ParamIOSubId_GET;
    reply.header.uuid    = uuid;
    reply.has_id         = true;
    reply.id             = paramId;
    reply.has_type       = true;
    reply.type           = type;
    reply.has_data       = true;
    reply.data.size      = static_cast<pb_size_t>( data.size() );
    std::memcpy( reply.data.bytes, data.constData(), static_cast<size_t>( data.size() ) );
    return encodeMessage( reply, ParamIOMessage_fields );
  }

  QByteArray consoleMessage( const QString &text )
  {
    ConsoleMessage message = ConsoleMessage_init_zero;
    message.header.msgId   = MsgId_MSG_TERMINAL;
    message.this_frame     = 1;
    message.total_frames   = 1;

    const QByteArray utf8 = text.toUtf8();
    message.data.size     = static_cast<pb_size_t>( qMin<qsizetype>( utf8.size(), sizeof( message.data.bytes ) ) );
    std::memcpy( message.data.bytes, utf8.constData(), static_cast<size_t>( message.data.size ) );
    return encodeMessage( message, ConsoleMessage_fields );
  }
}    // namespace orbit::test
