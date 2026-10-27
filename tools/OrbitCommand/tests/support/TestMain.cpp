#include <QCoreApplication>
#include <QStandardPaths>
#include <QTemporaryDir>

#include <gtest/gtest.h>

int main( int argc, char *argv[] )
{
  static QTemporaryDir settingsDirectory;
  if( settingsDirectory.isValid() )
  {
    qputenv( "XDG_CONFIG_HOME", settingsDirectory.path().toUtf8() );
  }

  QStandardPaths::setTestModeEnabled( true );

  QCoreApplication application( argc, argv );
  QCoreApplication::setOrganizationName( "OrbitESC" );
  QCoreApplication::setOrganizationDomain( "orbitesc.local" );
  QCoreApplication::setApplicationName( "OrbitCommandTests" );
  QCoreApplication::setApplicationVersion( "0.1.0" );

  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
