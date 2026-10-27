#include <gtest/gtest.h>

#include <QSettings>

#include "app/AppController.h"
#include "device/SimulatorDefaults.h"

using namespace orbit::app;

namespace
{
  class AppControllerTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      QSettings settings;
      settings.clear();
      settings.sync();
    }

    static QVariantMap simulatorEntry( const QVariantList &targets )
    {
      for( const QVariant &target : targets )
      {
        const QVariantMap entry = target.toMap();
        if( entry.value( "kind" ).toString() == QStringLiteral( "simulator" ) )
        {
          return entry;
        }
      }
      return {};
    }
  };
}    // namespace

TEST_F( AppControllerTest, DiscoversSimulatorTargetWithDefaults )
{
  AppController controller;
  const QVariantMap simulator = simulatorEntry( controller.availableTargets() );

  ASSERT_FALSE( simulator.isEmpty() );
  EXPECT_EQ( simulator.value( "id" ).toString(), QStringLiteral( "simulator" ) );
  EXPECT_EQ( simulator.value( "host" ).toString(),
             QString::fromLatin1( orbit::device::DefaultSimulatorHost ) );
  EXPECT_EQ( simulator.value( "port" ).toInt(), orbit::device::DefaultSimulatorPort );
}

TEST_F( AppControllerTest, SelectsAnAvailableTargetByDefault )
{
  AppController controller;
  ASSERT_FALSE( controller.selectedTargetId().isEmpty() );

  bool found = false;
  for( const QVariant &target : controller.availableTargets() )
  {
    if( target.toMap().value( "id" ).toString() == controller.selectedTargetId() )
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE( found );
}

TEST_F( AppControllerTest, PersistsSimulatorEndpoint )
{
  {
    AppController controller;
    controller.setSimulatorHost( QStringLiteral( "192.0.2.10" ) );
    controller.setSimulatorPort( 41000 );
    EXPECT_EQ( controller.simulatorHost(), QStringLiteral( "192.0.2.10" ) );
    EXPECT_EQ( controller.simulatorPort(), 41000 );
  }

  AppController reloaded;
  EXPECT_EQ( reloaded.simulatorHost(), QStringLiteral( "192.0.2.10" ) );
  EXPECT_EQ( reloaded.simulatorPort(), 41000 );

  const QVariantMap simulator = simulatorEntry( reloaded.availableTargets() );
  ASSERT_FALSE( simulator.isEmpty() );
  EXPECT_EQ( simulator.value( "host" ).toString(), QStringLiteral( "192.0.2.10" ) );
  EXPECT_EQ( simulator.value( "port" ).toInt(), 41000 );
}

TEST_F( AppControllerTest, BoundsSimulatorPort )
{
  AppController controller;

  controller.setSimulatorPort( 70000 );
  EXPECT_EQ( controller.simulatorPort(), 65535 );

  controller.setSimulatorPort( -5 );
  EXPECT_EQ( controller.simulatorPort(), 1 );
}

TEST_F( AppControllerTest, PersistsSelectedTarget )
{
  {
    AppController controller;
    controller.selectTarget( QStringLiteral( "simulator" ) );
    EXPECT_EQ( controller.selectedTargetId(), QStringLiteral( "simulator" ) );
    EXPECT_TRUE( controller.simulatorTargetSelected() );
  }

  AppController reloaded;
  EXPECT_EQ( reloaded.selectedTargetId(), QStringLiteral( "simulator" ) );
  EXPECT_TRUE( reloaded.simulatorTargetSelected() );
}

TEST_F( AppControllerTest, PersistsSelectedPageAndPlotScale )
{
  {
    AppController controller;
    controller.setSelectedPage( 2 );
    controller.setPlotAutoScale( false );
  }

  AppController reloaded;
  EXPECT_EQ( reloaded.selectedPage(), 2 );
  EXPECT_FALSE( reloaded.plotAutoScale() );
}

TEST_F( AppControllerTest, RejectsOutOfRangePage )
{
  AppController controller;
  controller.setSelectedPage( 99 );
  EXPECT_EQ( controller.selectedPage(), 3 );

  controller.setSelectedPage( -3 );
  EXPECT_EQ( controller.selectedPage(), 0 );
}
