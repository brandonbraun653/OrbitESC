#include "device/SerialTransportProvider.h"

#include "device/SerialTransport.h"

#include <QSerialPortInfo>

namespace orbit::device
{
  QString SerialTransportProvider::kind() const
  {
    return QStringLiteral( "serial" );
  }

  QList<ConnectionTarget> SerialTransportProvider::discover() const
  {
    QList<ConnectionTarget> targets;
    const auto serialPorts = QSerialPortInfo::availablePorts();
    for( const QSerialPortInfo &info : serialPorts )
    {
      const quint16 vid = info.hasVendorIdentifier() ? info.vendorIdentifier() : 0;
      const quint16 pid = info.hasProductIdentifier() ? info.productIdentifier() : 0;
      const bool likelyOrbit = vid == 0xCAFE || info.description().contains( "Orbit", Qt::CaseInsensitive ) ||
                               info.manufacturer().contains( "Orbit", Qt::CaseInsensitive );

      ConnectionTarget target;
      target.kind = kind();
      target.id = QStringLiteral( "serial:%1" ).arg( info.portName() );
      target.displayName = info.portName();
      target.properties.insert( "portName", info.portName() );
      target.properties.insert( "description", info.description() );
      target.properties.insert( "manufacturer", info.manufacturer() );
      target.properties.insert( "vid", vid );
      target.properties.insert( "pid", pid );
      target.properties.insert( "likelyOrbit", likelyOrbit );
      targets.append( target );
    }
    return targets;
  }

  Transport *SerialTransportProvider::create( const ConnectionTarget &target, QObject *parent ) const
  {
    if( target.kind != kind() )
    {
      return nullptr;
    }
    return new SerialTransport( parent );
  }
}    // namespace orbit::device
