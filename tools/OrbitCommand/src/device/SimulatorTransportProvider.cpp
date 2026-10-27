#include "device/SimulatorTransportProvider.h"

#include "device/SimulatorDefaults.h"
#include "device/TcpTransport.h"

namespace orbit::device
{
  QString SimulatorTransportProvider::kind() const
  {
    return QStringLiteral( "simulator" );
  }

  QList<ConnectionTarget> SimulatorTransportProvider::discover() const
  {
    ConnectionTarget target;
    target.kind = kind();
    target.id = kind();
    target.displayName = QStringLiteral( "Simulator" );
    target.properties.insert( "host", QString::fromLatin1( DefaultSimulatorHost ) );
    target.properties.insert( "port", DefaultSimulatorPort );
    return { target };
  }

  Transport *SimulatorTransportProvider::create( const ConnectionTarget &target, QObject *parent ) const
  {
    if( target.kind != kind() )
    {
      return nullptr;
    }
    return new TcpTransport( parent );
  }
}    // namespace orbit::device
