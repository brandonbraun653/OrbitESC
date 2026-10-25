#pragma once

#include <QByteArray>
#include <QString>
#include <QVariant>

#include "system_config.pb.h"

namespace orbit::protocol
{
  QString parameterTypeName( ParamType type );
  bool isSupportedParameterType( ParamType type );

  QVariant decodeParameterValue( ParamType type, const QByteArray &data, bool *ok = nullptr );
  QByteArray encodeParameterValue( ParamType type, const QVariant &value, bool *ok = nullptr );
}    // namespace orbit::protocol
