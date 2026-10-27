#pragma once

#include <QList>
#include <QString>

#include "device/ConnectionTarget.h"
#include "device/Transport.h"

namespace orbit::device
{
  class TransportProvider
  {
  public:
    virtual ~TransportProvider() = default;

    virtual QString kind() const = 0;
    virtual QList<ConnectionTarget> discover() const = 0;
    virtual Transport *create( const ConnectionTarget &target, QObject *parent ) const = 0;
  };
}    // namespace orbit::device
