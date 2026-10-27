#pragma once

#include <QByteArray>
#include <QProcess>
#include <QString>

namespace orbit::test
{
  class SimulatorProcess
  {
  public:
    explicit SimulatorProcess( QObject *parent = nullptr );
    ~SimulatorProcess();

    static QString resolveBinary();
    static quint16 findFreePort();

    bool start( quint16 port, int timeoutMs = 20000 );
    bool restart( int timeoutMs = 20000 );
    void stop();

    bool isRunning() const;
    quint16 port() const
    {
      return m_port;
    }
    QString logText() const;
    void setLogFile( const QString &path );
    bool waitForReady( int timeoutMs = 20000 );

  private:
    void captureOutput();

    QProcess m_process;
    quint16 m_port = 0;
    QByteArray m_log;
    QString m_logFile;
  };
}    // namespace orbit::test
