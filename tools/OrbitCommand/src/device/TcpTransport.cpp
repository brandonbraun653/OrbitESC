#include "device/TcpTransport.h"

namespace orbit::device
{
  namespace
  {
    constexpr int ConnectTimeoutMs = 3000;
    constexpr int RetryDelaysMs[]  = { 500, 1000, 2000, 5000 };
  }

  TcpTransport::TcpTransport( QObject *parent ) :
      Transport( parent ),
      m_socket( new QTcpSocket( this ) ),
      m_retryTimer( new QTimer( this ) ),
      m_connectTimer( new QTimer( this ) )
  {
    m_retryTimer->setSingleShot( true );
    m_connectTimer->setSingleShot( true );

    connect( m_socket, &QTcpSocket::connected, this, &TcpTransport::handleConnected );
    connect( m_socket, &QTcpSocket::disconnected, this, &TcpTransport::handleDisconnected );
    connect( m_socket, &QTcpSocket::readyRead, this, &TcpTransport::handleReadyRead );
    connect( m_socket, &QTcpSocket::errorOccurred, this, &TcpTransport::handleSocketError );
    connect( m_retryTimer, &QTimer::timeout, this, &TcpTransport::attemptConnect );
    connect( m_connectTimer, &QTimer::timeout, this, &TcpTransport::handleConnectTimeout );
  }

  bool TcpTransport::open( const ConnectionTarget &target )
  {
    const QString host = target.properties.value( "host" ).toString().trimmed();
    bool portOk = false;
    const uint port = target.properties.value( "port" ).toUInt( &portOk );
    if( host.isEmpty() || !portOk || port == 0 || port > 65535 )
    {
      emit errorOccurred( tr( "Simulator target has an invalid host or port" ) );
      return false;
    }

    m_userClosed = true;
    stopTimers();
    m_socket->abort();

    m_host = host;
    m_port = static_cast<quint16>( port );
    m_retryIndex = 0;
    m_errorReported = false;
    m_userClosed = false;

    attemptConnect();
    return true;
  }

  void TcpTransport::close()
  {
    m_userClosed = true;
    stopTimers();
    if( m_socket->state() != QAbstractSocket::UnconnectedState )
    {
      m_socket->abort();
    }
    emit closed();
  }

  void TcpTransport::write( const QByteArray &data )
  {
    if( !isOpen() )
    {
      emit errorOccurred( tr( "Simulator socket is not connected" ) );
      return;
    }

    if( m_socket->write( data ) != data.size() )
    {
      emit errorOccurred( tr( "Simulator write failed: %1" ).arg( m_socket->errorString() ) );
    }
  }

  bool TcpTransport::isOpen() const
  {
    return m_socket->state() == QAbstractSocket::ConnectedState;
  }

  void TcpTransport::handleConnected()
  {
    m_connectTimer->stop();
    m_retryTimer->stop();
    m_retryIndex = 0;
    m_errorReported = false;
    emit opened();
  }

  void TcpTransport::handleDisconnected()
  {
    if( m_userClosed )
    {
      return;
    }

    scheduleRetry();
  }

  void TcpTransport::handleReadyRead()
  {
    const QByteArray data = m_socket->readAll();
    if( !data.isEmpty() )
    {
      emit dataReceived( data );
    }
  }

  void TcpTransport::handleSocketError( QAbstractSocket::SocketError error )
  {
    Q_UNUSED( error )

    if( m_userClosed )
    {
      return;
    }

    m_connectTimer->stop();
    reportErrorOnce( tr( "Simulator connection failed: %1" ).arg( m_socket->errorString() ) );
    scheduleRetry();
  }

  void TcpTransport::handleConnectTimeout()
  {
    if( m_userClosed )
    {
      return;
    }

    reportErrorOnce( tr( "Timed out connecting to simulator at %1:%2" ).arg( m_host ).arg( m_port ) );
    m_socket->abort();
    scheduleRetry();
  }

  void TcpTransport::attemptConnect()
  {
    if( m_userClosed || m_socket->state() == QAbstractSocket::ConnectedState )
    {
      return;
    }

    if( m_socket->state() != QAbstractSocket::UnconnectedState )
    {
      m_socket->abort();
    }

    m_socket->connectToHost( m_host, m_port );
    m_connectTimer->start( ConnectTimeoutMs );
  }

  void TcpTransport::stopTimers()
  {
    m_retryTimer->stop();
    m_connectTimer->stop();
  }

  void TcpTransport::scheduleRetry()
  {
    if( m_userClosed || m_retryTimer->isActive() )
    {
      return;
    }

    const int delay = RetryDelaysMs[qMin( m_retryIndex, 3 )];
    m_retryIndex++;
    m_retryTimer->start( delay );
  }

  void TcpTransport::reportErrorOnce( const QString &message )
  {
    if( m_errorReported )
    {
      return;
    }

    m_errorReported = true;
    emit errorOccurred( message );
  }
}    // namespace orbit::device
