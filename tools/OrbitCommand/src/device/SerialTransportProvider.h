#pragma once

#include "device/TransportProvider.h"

namespace orbit::device
{
  class SerialTransportProvider : public TransportProvider
  {
  public:
    QString kind() const override;
    QList<ConnectionTarget> discover() const override;
    Transport *create( const ConnectionTarget &target, QObject *parent ) const override;
  };
}    // namespace orbit::device
