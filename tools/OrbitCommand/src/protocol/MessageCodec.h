#pragma once

#include <QByteArray>
#include <QString>
#include <optional>

#include <pb_decode.h>
#include <pb_encode.h>

#include "motor_control.pb.h"
#include "serial_interface.pb.h"
#include "system_config.pb.h"
#include "system_control.pb.h"
#include "system_data.pb.h"

namespace orbit::protocol
{
  struct HeaderView
  {
    quint8 msgId  = 0;
    quint8 subId  = 0;
    quint16 uuid  = 0;
  };

  template<typename T>
  QByteArray encodeMessage( const T &message, const pb_msgdesc_t *fields )
  {
    size_t encoded_size = 0;
    if( !pb_get_encoded_size( &encoded_size, fields, &message ) )
    {
      return {};
    }

    QByteArray output;
    output.resize( static_cast<int>( encoded_size ) );

    pb_ostream_t stream = pb_ostream_from_buffer( reinterpret_cast<pb_byte_t *>( output.data() ), encoded_size );
    if( !pb_encode( &stream, fields, &message ) )
    {
      return {};
    }

    return output;
  }

  template<typename T>
  std::optional<T> decodeMessage( const QByteArray &payload, const pb_msgdesc_t *fields )
  {
    T message{};
    pb_istream_t stream = pb_istream_from_buffer( reinterpret_cast<const pb_byte_t *>( payload.constData() ),
                                                  static_cast<size_t>( payload.size() ) );
    if( !pb_decode( &stream, fields, &message ) )
    {
      return std::nullopt;
    }

    return message;
  }

  std::optional<HeaderView> peekHeader( const QByteArray &payload );

  QByteArray encodePing( quint16 uuid );
  QByteArray encodeSystemControl( SystemControlSubId subId, quint16 uuid );
  QByteArray encodeParamGet( ParamId paramId, quint16 uuid );
  QByteArray encodeParamSet( ParamId paramId, ParamType type, const QByteArray &data, quint16 uuid );

  struct SystemVoltages
  {
    float vMcu   = 0.0f;
    float vDcLink = 0.0f;
    float vTemp  = 0.0f;
    float vIsense = 0.0f;
  };

  struct PhaseCurrents
  {
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
  };

  std::optional<SystemVoltages> decodeSystemVoltages( const SystemDataMessage &message );
  std::optional<PhaseCurrents> decodePhaseCurrents( const SystemDataMessage &message );

  inline const char *statusCodeName( StatusCode code )
  {
    switch( code )
    {
      case StatusCode_NO_ERROR:
        return "NO_ERROR";
      case StatusCode_INVALID_PARAM:
        return "INVALID_PARAM";
      case StatusCode_INVALID_TYPE:
        return "INVALID_TYPE";
      case StatusCode_INVALID_VALUE:
        return "INVALID_VALUE";
      case StatusCode_REQUEST_FAILED:
        return "REQUEST_FAILED";
      case StatusCode_UNKNOWN_ERROR:
      default:
        return "UNKNOWN_ERROR";
    }
  }
}    // namespace orbit::protocol
