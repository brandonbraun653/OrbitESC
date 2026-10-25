#include "protocol/MessageCodec.h"

namespace orbit::protocol
{
  std::optional<HeaderView> peekHeader( const QByteArray &payload )
  {
    const auto base = decodeMessage<BaseMessage>( payload, BaseMessage_fields );
    if( !base.has_value() )
    {
      return std::nullopt;
    }

    HeaderView header;
    header.msgId = base->header.msgId;
    header.subId = base->header.subId;
    header.uuid  = base->header.uuid;
    return header;
  }

  QByteArray encodePing( quint16 uuid )
  {
    PingMessage message = PingMessage_init_zero;
    message.header.msgId = MsgId_MSG_PING_CMD;
    message.header.subId = SubId_SUB_MSG_NONE;
    message.header.uuid  = uuid;
    return encodeMessage( message, PingMessage_fields );
  }

  QByteArray encodeSystemControl( SystemControlSubId subId, quint16 uuid )
  {
    SystemControlMessage message = SystemControlMessage_init_zero;
    message.header.msgId = MsgId_MSG_SYS_CTRL;
    message.header.subId = static_cast<quint8>( subId );
    message.header.uuid  = uuid;
    return encodeMessage( message, SystemControlMessage_fields );
  }

  QByteArray encodeParamGet( ParamId paramId, quint16 uuid )
  {
    ParamIOMessage message = ParamIOMessage_init_zero;
    message.header.msgId  = MsgId_MSG_PARAM_IO;
    message.header.subId  = ParamIOSubId_GET;
    message.header.uuid   = uuid;
    message.has_id        = true;
    message.id            = paramId;
    return encodeMessage( message, ParamIOMessage_fields );
  }

  QByteArray encodeParamSet( ParamId paramId, ParamType type, const QByteArray &data, quint16 uuid )
  {
    ParamIOMessage message = ParamIOMessage_init_zero;
    if( data.size() > static_cast<int>( sizeof( message.data.bytes ) ) )
    {
      return {};
    }

    message.header.msgId  = MsgId_MSG_PARAM_IO;
    message.header.subId  = ParamIOSubId_SET;
    message.header.uuid   = uuid;
    message.has_id        = true;
    message.id            = paramId;
    message.has_type      = true;
    message.type          = type;
    message.has_data      = true;
    message.data.size     = static_cast<pb_size_t>( data.size() );
    memcpy( message.data.bytes, data.constData(), static_cast<size_t>( data.size() ) );
    return encodeMessage( message, ParamIOMessage_fields );
  }

  std::optional<SystemVoltages> decodeSystemVoltages( const SystemDataMessage &message )
  {
    ADCSystemVoltagesPayload payload{};
    pb_istream_t stream = pb_istream_from_buffer( message.payload.bytes, message.payload.size );
    if( !pb_decode( &stream, ADCSystemVoltagesPayload_fields, &payload ) )
    {
      return std::nullopt;
    }

    return SystemVoltages{ payload.v_mcu, payload.v_dc_link, payload.v_temp, payload.v_isense };
  }

  std::optional<PhaseCurrents> decodePhaseCurrents( const SystemDataMessage &message )
  {
    ADCPhaseCurrentsPayload payload{};
    pb_istream_t stream = pb_istream_from_buffer( message.payload.bytes, message.payload.size );
    if( !pb_decode( &stream, ADCPhaseCurrentsPayload_fields, &payload ) )
    {
      return std::nullopt;
    }

    return PhaseCurrents{ payload.ia, payload.ib, payload.ic };
  }
}    // namespace orbit::protocol
