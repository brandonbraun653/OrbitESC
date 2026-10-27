#pragma once

#include <QMetaType>
#include <QString>
#include <QVariantMap>

namespace orbit::device
{
  struct ConnectionTarget
  {
    QString kind;
    QString id;
    QString displayName;
    QVariantMap properties;
  };
}    // namespace orbit::device

Q_DECLARE_METATYPE( orbit::device::ConnectionTarget )
