#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include "device/SimulatorDefaults.h"
#include "device/TcpTransport.h"
#include "device/TransportRegistry.h"
#include "support/FakeTransport.h"
#include "support/MockTransportProvider.h"

using namespace orbit::device;
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

TEST( TransportRegistry, SimulatorTargetIsAlwaysDiscovered )
{
  TransportRegistry registry;
  const QList<ConnectionTarget> targets = registry.discoverTargets();

  bool found = false;
  for( const ConnectionTarget &target : targets )
  {
    if( target.kind == QStringLiteral( "simulator" ) )
    {
      found = true;
      EXPECT_EQ( target.id, QStringLiteral( "simulator" ) );
      EXPECT_EQ( target.properties.value( "host" ).toString(), QString::fromLatin1( DefaultSimulatorHost ) );
      EXPECT_EQ( target.properties.value( "port" ).toUInt(), static_cast<uint>( DefaultSimulatorPort ) );
    }
  }

  EXPECT_TRUE( found );
}

TEST( TransportRegistry, CreatesTransportForKnownKind )
{
  TransportRegistry registry;
  ConnectionTarget target;
  target.kind = QStringLiteral( "simulator" );
  target.id   = QStringLiteral( "simulator" );
  target.properties.insert( "host", QStringLiteral( "localhost" ) );
  target.properties.insert( "port", DefaultSimulatorPort );

  std::unique_ptr<Transport> transport( registry.create( target, nullptr ) );
  ASSERT_NE( transport, nullptr );
  EXPECT_NE( qobject_cast<TcpTransport *>( transport.get() ), nullptr );
}

TEST( TransportRegistry, RejectsUnknownKind )
{
  TransportRegistry registry;
  ConnectionTarget target;
  target.kind = QStringLiteral( "unknown" );
  target.id   = QStringLiteral( "unknown" );

  EXPECT_EQ( registry.create( target, nullptr ), nullptr );
}

TEST( TransportRegistry, DelegatesDiscoveryAndCreationToProviders )
{
  TransportRegistry registry;

  auto provider = std::make_unique<NiceMock<orbit::test::MockTransportProvider>>();
  auto *mock    = provider.get();

  ConnectionTarget discovered;
  discovered.kind        = QStringLiteral( "mock" );
  discovered.id          = QStringLiteral( "mock:one" );
  discovered.displayName = QStringLiteral( "Mock One" );

  EXPECT_CALL( *mock, discover() ).WillOnce( Return( QList<ConnectionTarget>{ discovered } ) );
  EXPECT_CALL( *mock, kind() ).WillRepeatedly( Return( QStringLiteral( "mock" ) ) );
  EXPECT_CALL( *mock, create( _, ::testing::IsNull() ) )
      .WillOnce( Return( new orbit::test::FakeTransport() ) );

  registry.addProvider( std::move( provider ) );

  const QList<ConnectionTarget> targets = registry.discoverTargets();
  bool found                            = false;
  for( const ConnectionTarget &target : targets )
  {
    if( target.id == discovered.id )
    {
      found = true;
    }
  }
  EXPECT_TRUE( found );

  ConnectionTarget requested = discovered;
  std::unique_ptr<Transport> transport( registry.create( requested, nullptr ) );
  EXPECT_NE( transport, nullptr );
}
