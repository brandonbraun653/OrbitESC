#pragma once

#include <QList>
#include <QString>

#include <memory>
#include <vector>

#include "device/ConnectionTarget.h"
#include "device/TransportProvider.h"

namespace orbit::device
{
  class TransportRegistry
  {
  public:
    TransportRegistry();
    ~TransportRegistry();

    TransportRegistry( const TransportRegistry & ) = delete;
    TransportRegistry &operator=( const TransportRegistry & ) = delete;

    void addProvider( std::unique_ptr<TransportProvider> provider );
    QList<ConnectionTarget> discoverTargets() const;
    Transport *create( const ConnectionTarget &target, QObject *parent ) const;

  private:
    std::vector<std::unique_ptr<TransportProvider>> m_providers;
  };
}    // namespace orbit::device
