#include "device/SerialTransport.h"

namespace orbit::device
{
  namespace
  {
    constexpr qint32 CommandBaudRate = 2000000;
  }

  SerialTransport::SerialTransport( QObject *parent ) : Transport( parent ), m_port( new QSerialPort( this ) )
  {
    connect( m_port, &QSerialPort::readyRead, this, &SerialTransport::handleReadyRead );
    connect( m_port, &QSerialPort::errorOccurred, this, &SerialTransport::handleError );
  }

  bool SerialTransport::open( const ConnectionTarget &target )
  {
    const QString portName = target.properties.value( "portName" ).toString();
    if( portName.isEmpty() )
    {
      emit errorOccurred( tr( "Serial target is missing a port name" ) );
      return false;
    }

    if( m_port->isOpen() )
    {
      m_port->close();
    }

    m_port->setPortName( portName );
    m_port->setBaudRate( CommandBaudRate );
    m_port->setDataBits( QSerialPort::Data8 );
    m_port->setParity( QSerialPort::NoParity );
    m_port->setStopBits( QSerialPort::OneStop );
    m_port->setFlowControl( QSerialPort::NoFlowControl );
    m_reportedError = false;

    if( !m_port->open( QIODevice::ReadWrite ) )
    {
      emit errorOccurred( tr( "Could not open %1: %2" ).arg( portName, m_port->errorString() ) );
      return false;
    }

    m_port->clear();
    emit opened();
    return true;
  }

  void SerialTransport::close()
  {
    if( m_port->isOpen() )
    {
      m_port->flush();
      m_port->close();
    }
    emit closed();
  }

  void SerialTransport::write( const QByteArray &data )
  {
    if( !m_port->isOpen() )
    {
      emit errorOccurred( tr( "Serial port is not open" ) );
      return;
    }

    const qint64 written = m_port->write( data );
    if( written != data.size() )
    {
      emit errorOccurred( tr( "Serial write failed: %1" ).arg( m_port->errorString() ) );
    }
  }

  bool SerialTransport::isOpen() const
  {
    return m_port->isOpen();
  }

  void SerialTransport::handleReadyRead()
  {
    const QByteArray data = m_port->readAll();
    if( !data.isEmpty() )
    {
      emit dataReceived( data );
    }
  }

  void SerialTransport::handleError( QSerialPort::SerialPortError error )
  {
    if( error == QSerialPort::NoError )
    {
      return;
    }

    if( !m_reportedError )
    {
      m_reportedError = true;
      emit errorOccurred( m_port->errorString() );
    }

    if( error == QSerialPort::ResourceError || error == QSerialPort::DeviceNotFoundError )
    {
      if( m_port->isOpen() )
      {
        m_port->close();
      }
      emit closed();
    }
  }
}    // namespace orbit::device
