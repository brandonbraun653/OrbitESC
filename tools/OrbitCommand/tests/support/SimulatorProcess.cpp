#include "support/SimulatorProcess.h"

#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QHostAddress>
#include <QTcpServer>
#include <QThread>

#include "support/Wait.h"

namespace orbit::test
{
  namespace
  {
    QString repositoryRoot()
    {
#ifdef ORBIT_REPO_ROOT_PATH
      return QString::fromUtf8( ORBIT_REPO_ROOT_PATH );
#else
      return QDir::currentPath();
#endif
    }
  }    // namespace

  SimulatorProcess::SimulatorProcess( QObject *parent )
  {
    Q_UNUSED( parent )
    m_process.setProcessChannelMode( QProcess::MergedChannels );
    QObject::connect( &m_process, &QProcess::readyRead, &m_process,
                      [this]()
                      {
                        captureOutput();
                      } );
  }

  SimulatorProcess::~SimulatorProcess()
  {
    stop();
  }

  QString SimulatorProcess::resolveBinary()
  {
    const QByteArray overrideBinary = qgetenv( "ORBIT_SIMULATOR_BINARY" );
    if( !overrideBinary.isEmpty() )
    {
      const QString path = QString::fromLocal8Bit( overrideBinary );
      if( QFileInfo::exists( path ) )
      {
        return path;
      }
    }

    const QString root = repositoryRoot();
    const QStringList candidates = {
      root + "/build/sim-release/bin/OrbitESC",
      root + "/build/sim/bin/OrbitESC",
    };

    for( const QString &candidate : candidates )
    {
      if( QFileInfo::exists( candidate ) )
      {
        return candidate;
      }
    }

    return {};
  }

  quint16 SimulatorProcess::findFreePort()
  {
    QTcpServer probe;
    if( !probe.listen( QHostAddress::LocalHost, 0 ) )
    {
      return 0;
    }

    const quint16 port = probe.serverPort();
    probe.close();
    return port;
  }

  bool SimulatorProcess::start( quint16 port, int timeoutMs )
  {
    stop();

    const QString binary = resolveBinary();
    if( binary.isEmpty() || port == 0 )
    {
      return false;
    }

    m_port = port;
    m_log.clear();

    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    environment.insert( "ORBITESC_SIM_PORT", QString::number( port ) );
    m_process.setProcessEnvironment( environment );
    m_process.start( binary, {} );

    if( !m_process.waitForStarted( 5000 ) )
    {
      stop();
      return false;
    }

    const bool ready = waitForReady( timeoutMs );
    if( !ready )
    {
      stop();
    }
    return ready;
  }

  bool SimulatorProcess::restart( int timeoutMs )
  {
    const quint16 port = m_port;
    stop();
    return start( port, timeoutMs );
  }

  void SimulatorProcess::stop()
  {
    if( m_process.state() == QProcess::NotRunning )
    {
      return;
    }

    m_process.terminate();
    if( !m_process.waitForFinished( 3000 ) )
    {
      m_process.kill();
      m_process.waitForFinished( 3000 );
    }

    captureOutput();
  }

  bool SimulatorProcess::isRunning() const
  {
    return m_process.state() != QProcess::NotRunning;
  }

  QString SimulatorProcess::logText() const
  {
    return QString::fromUtf8( m_log );
  }

  void SimulatorProcess::setLogFile( const QString &path )
  {
    m_logFile = path;
  }

  bool SimulatorProcess::waitForReady( int timeoutMs )
  {
    QElapsedTimer timer;
    timer.start();

    const QByteArray marker = QByteArray( "Created server on port " ) + QByteArray::number( m_port );

    while( timer.elapsed() < timeoutMs )
    {
      captureOutput();

      if( m_process.state() == QProcess::NotRunning )
      {
        return false;
      }

      if( m_log.contains( marker ) )
      {
        // The listener exists, but the simulator's accept loop still needs a
        // scheduling pass before the real client connects.
        processEventsFor( 100 );
        return true;
      }

      processEventsFor( 50 );
    }

    return false;
  }

  void SimulatorProcess::captureOutput()
  {
    const QByteArray chunk = m_process.readAll();
    if( chunk.isEmpty() )
    {
      return;
    }

    m_log.append( chunk );
    if( !m_logFile.isEmpty() )
    {
      QFile file( m_logFile );
      if( file.open( QIODevice::Append | QIODevice::WriteOnly ) )
      {
        file.write( chunk );
      }
    }
  }
}    // namespace orbit::test
