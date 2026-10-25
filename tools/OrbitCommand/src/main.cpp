#include "app/AppController.h"
#include "model/LogModel.h"
#include "model/ParameterModel.h"
#include "model/TelemetryBuffer.h"

#include <QColor>
#include <QFile>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QImage>
#include <QPalette>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QSettings>
#include <QTimer>

int main( int argc, char *argv[] )
{
  QQuickStyle::setStyle( "Fusion" );

  QGuiApplication app( argc, argv );
  QCoreApplication::setOrganizationName( "OrbitESC" );
  QCoreApplication::setOrganizationDomain( "orbitesc.local" );
  QCoreApplication::setApplicationName( "OrbitCommand" );
  QCoreApplication::setApplicationVersion( "0.1.0" );

  const QStringList fontFiles = {
    QStringLiteral( ":/assets/fonts/Orbitron-VariableFont_wght.ttf" ),
    QStringLiteral( ":/assets/fonts/JetBrainsMono-Regular.ttf" ),
    QStringLiteral( ":/assets/fonts/JetBrainsMono-Bold.ttf" ),
  };
  for( const QString &fontFile : fontFiles )
  {
    if( QFile::exists( fontFile ) )
    {
      QFontDatabase::addApplicationFont( fontFile );
    }
  }

  QPalette palette;
  palette.setColor( QPalette::Window, QColor( "#070A12" ) );
  palette.setColor( QPalette::WindowText, QColor( "#E6F1FF" ) );
  palette.setColor( QPalette::Base, QColor( "#0E1420" ) );
  palette.setColor( QPalette::AlternateBase, QColor( "#131C2A" ) );
  palette.setColor( QPalette::ToolTipBase, QColor( "#131C2A" ) );
  palette.setColor( QPalette::ToolTipText, QColor( "#E6F1FF" ) );
  palette.setColor( QPalette::Text, QColor( "#E6F1FF" ) );
  palette.setColor( QPalette::Button, QColor( "#0E1420" ) );
  palette.setColor( QPalette::ButtonText, QColor( "#E6F1FF" ) );
  palette.setColor( QPalette::BrightText, QColor( "#FF1744" ) );
  palette.setColor( QPalette::Highlight, QColor( "#00E5FF" ) );
  palette.setColor( QPalette::HighlightedText, QColor( "#031018" ) );
  palette.setColor( QPalette::PlaceholderText, QColor( "#7C8DA6" ) );
  app.setPalette( palette );

  qmlRegisterUncreatableType<orbit::model::ParameterModel>( "OrbitCommand", 1, 0, "ParameterModel",
                                                            "Provided by AppController" );
  qmlRegisterUncreatableType<orbit::model::TelemetryBuffer>( "OrbitCommand", 1, 0, "TelemetryBuffer",
                                                             "Provided by AppController" );
  qmlRegisterUncreatableType<orbit::model::LogModel>( "OrbitCommand", 1, 0, "LogModel", "Provided by AppController" );

  orbit::app::AppController controller;
  const int pageIndex = app.arguments().indexOf( "--page" );
  if( pageIndex >= 0 && pageIndex + 1 < app.arguments().size() )
  {
    controller.setSelectedPage( app.arguments().at( pageIndex + 1 ).toInt() );
  }

  const int connectIndex = app.arguments().indexOf( "--connect" );
  if( connectIndex >= 0 && connectIndex + 1 < app.arguments().size() )
  {
    controller.selectPort( app.arguments().at( connectIndex + 1 ) );
    QTimer::singleShot( 200, &controller, [&controller]() { controller.connectDevice(); } );
  }

  if( app.arguments().contains( "--stream" ) )
  {
    QObject::connect( &controller, &orbit::app::AppController::connectionStateChanged, &controller,
                      [&controller]()
                      {
                        if( controller.online() && !controller.streamEnabled() )
                        {
                          controller.setPhaseCurrentStream( true );
                        }
                      } );
  }

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty( "app", &controller );

  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreationFailed, &app,
      []()
      {
        QCoreApplication::exit( 1 );
      },
      Qt::QueuedConnection );

  engine.loadFromModule( "OrbitCommand", "Main" );
  if( engine.rootObjects().isEmpty() )
  {
    return 1;
  }

  if( app.arguments().contains( "--smoke-test" ) )
  {
    QTimer::singleShot( 500, &app, &QCoreApplication::quit );
  }

  auto *rootWindow = qobject_cast<QQuickWindow *>( engine.rootObjects().first() );
  const int sizeIndex = app.arguments().indexOf( "--size" );
  if( rootWindow && sizeIndex >= 0 && sizeIndex + 1 < app.arguments().size() )
  {
    const QStringList dimensions = app.arguments().at( sizeIndex + 1 ).split( 'x' );
    if( dimensions.size() == 2 )
    {
      rootWindow->resize( dimensions.at( 0 ).toInt(), dimensions.at( 1 ).toInt() );
    }
  }

  const int screenshotIndex = app.arguments().indexOf( "--screenshot" );
  if( screenshotIndex >= 0 && screenshotIndex + 1 < app.arguments().size() )
  {
    const QString path = app.arguments().at( screenshotIndex + 1 );
    const int delayIndex = app.arguments().indexOf( "--screenshot-delay" );
    const int delayMs = delayIndex >= 0 && delayIndex + 1 < app.arguments().size()
                            ? app.arguments().at( delayIndex + 1 ).toInt()
                            : 2500;
    QTimer::singleShot(
        delayMs, &app,
        [&app, rootWindow, path]()
        {
          if( rootWindow )
          {
            rootWindow->grabWindow().save( path );
          }
          app.quit();
        } );
  }

  return app.exec();
}
