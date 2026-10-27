#pragma once

#include "device/Transport.h"

#include <QSerialPort>

namespace orbit::device
{
  class SerialTransport : public Transport
  {
    Q_OBJECT
  public:
    explicit SerialTransport( QObject *parent = nullptr );

    bool open( const ConnectionTarget &target ) override;
    void close() override;
    void write( const QByteArray &data ) override;
    bool isOpen() const override;

  private slots:
    void handleReadyRead();
    void handleError( QSerialPort::SerialPortError error );

  private:
    QSerialPort *m_port = nullptr;
    bool m_reportedError = false;
  };
}    // namespace orbit::device
