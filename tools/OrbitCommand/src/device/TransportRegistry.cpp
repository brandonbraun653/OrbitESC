#include "device/TransportRegistry.h"

#include "device/SerialTransportProvider.h"
#include "device/SimulatorTransportProvider.h"

#include <utility>

namespace orbit::device
{
  TransportRegistry::TransportRegistry()
  {
    addProvider( std::make_unique<SerialTransportProvider>() );
    addProvider( std::make_unique<SimulatorTransportProvider>() );
  }

  TransportRegistry::~TransportRegistry() = default;

  void TransportRegistry::addProvider( std::unique_ptr<TransportProvider> provider )
  {
    if( provider )
    {
      m_providers.push_back( std::move( provider ) );
    }
  }

  QList<ConnectionTarget> TransportRegistry::discoverTargets() const
  {
    QList<ConnectionTarget> targets;
    for( const auto &provider : m_providers )
    {
      targets.append( provider->discover() );
    }
    return targets;
  }

  Transport *TransportRegistry::create( const ConnectionTarget &target, QObject *parent ) const
  {
    for( const auto &provider : m_providers )
    {
      if( provider->kind() == target.kind )
      {
        return provider->create( target, parent );
      }
    }
    return nullptr;
  }
}    // namespace orbit::device
