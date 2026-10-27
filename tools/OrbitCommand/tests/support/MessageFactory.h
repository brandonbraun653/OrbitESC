#pragma once

#include <QByteArray>

#include "protocol/FrameCodec.h"
#include "protocol/MessageCodec.h"

namespace orbit::test
{
  QByteArray frame( const QByteArray &payload );
  QByteArray ackFor( const orbit::protocol::HeaderView &header );
  QByteArray nackFor( const orbit::protocol::HeaderView &header, StatusCode status = StatusCode_REQUEST_FAILED );
  QByteArray tickMessage( quint32 tick );
  QByteArray statusMessage( MotorCtrlState state, quint32 tick );
  QByteArray voltagesMessage( float vMcu, float vDcLink, float vTemp, float vIsense );
  QByteArray phaseCurrentsMessage( float ia, float ib, float ic, quint32 timestampUs );
  QByteArray parameterReply( quint16 uuid, ParamId paramId, ParamType type, const QByteArray &data );
  QByteArray consoleMessage( const QString &text );
}    // namespace orbit::test
