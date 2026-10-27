#pragma once

#include "device/Transport.h"

#include <QAbstractSocket>
#include <QTcpSocket>
#include <QTimer>

namespace orbit::device
{
  class TcpTransport : public Transport
  {
    Q_OBJECT
  public:
    explicit TcpTransport( QObject *parent = nullptr );

    bool open( const ConnectionTarget &target ) override;
    void close() override;
    void write( const QByteArray &data ) override;
    bool isOpen() const override;

  private slots:
    void handleConnected();
    void handleDisconnected();
    void handleReadyRead();
    void handleSocketError( QAbstractSocket::SocketError error );
    void handleConnectTimeout();
    void attemptConnect();

  private:
    void stopTimers();
    void scheduleRetry();
    void reportErrorOnce( const QString &message );

    QTcpSocket *m_socket = nullptr;
    QTimer *m_retryTimer = nullptr;
    QTimer *m_connectTimer = nullptr;
    QString m_host;
    quint16 m_port = 0;
    int m_retryIndex = 0;
    bool m_userClosed = true;
    bool m_errorReported = false;
  };
}    // namespace orbit::device
