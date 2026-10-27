#include "support/FakeDeviceServer.h"

#include <QTcpSocket>

#include "support/MessageFactory.h"

namespace orbit::test
{
  using namespace orbit::protocol;

  FakeDeviceServer::FakeDeviceServer( QObject *parent ) : QTcpServer( parent )
  {
    connect( this, &QTcpServer::newConnection, this, &FakeDeviceServer::acceptConnection );

    m_tickTimer.setInterval( 100 );
    m_statusTimer.setInterval( 200 );
    m_streamTimer.setInterval( 33 );

    connect( &m_tickTimer, &QTimer::timeout, this, &FakeDeviceServer::sendTick );
    connect( &m_statusTimer, &QTimer::timeout, this, &FakeDeviceServer::sendStatus );
    connect( &m_streamTimer, &QTimer::timeout, this, &FakeDeviceServer::sendStreamSample );
  }

  bool FakeDeviceServer::start( quint16 port )
  {
    return listen( QHostAddress::LocalHost, port );
  }

  quint16 FakeDeviceServer::port() const
  {
    return serverPort();
  }

  void FakeDeviceServer::setTicksEnabled( bool enabled )
  {
    m_ticksEnabled = enabled;
    if( enabled && m_peer )
    {
      m_tickTimer.start();
    }
    else
    {
      m_tickTimer.stop();
    }
  }

  void FakeDeviceServer::setTickIntervalMs( int intervalMs )
  {
    m_tickTimer.setInterval( intervalMs );
  }

  void FakeDeviceServer::setStatusIntervalMs( int intervalMs )
  {
    m_statusTimer.setInterval( intervalMs );
  }

  void FakeDeviceServer::setStreamEnabled( bool enabled )
  {
    if( enabled )
    {
      m_streamTimer.start();
    }
    else
    {
      m_streamTimer.stop();
    }
  }

  void FakeDeviceServer::setStreamIntervalMs( int intervalMs )
  {
    m_streamTimer.setInterval( intervalMs );
  }

  void FakeDeviceServer::setParameter( int paramId, ParamType type, const QVariant &value )
  {
    m_parameterTypes.insert( paramId, type );
    m_parameterValues.insert( paramId, value );
  }

  void FakeDeviceServer::setMotorState( MotorCtrlState state )
  {
    m_motorState = state;
  }

  void FakeDeviceServer::acceptConnection()
  {
    m_peer = nextPendingConnection();
    if( !m_peer )
    {
      return;
    }

    connect( m_peer, &QTcpSocket::readyRead, this, &FakeDeviceServer::readFromClient );
    connect( m_peer, &QTcpSocket::disconnected, this,
             [this]()
             {
               stopTimers();
               m_peer = nullptr;
             } );

    m_tick = 0;
    if( m_ticksEnabled )
    {
      m_tickTimer.start();
    }
    m_statusTimer.start();
  }

  void FakeDeviceServer::readFromClient()
  {
    if( !m_peer )
    {
      return;
    }

    m_accumulator.append( m_peer->readAll() );
    const auto frames = m_accumulator.takeFrames();
    for( const QByteArray &payload : frames.payloads )
    {
      m_receivedPayloads.append( payload );
      processPayload( payload );
    }
  }

  void FakeDeviceServer::processPayload( const QByteArray &payload )
  {
    const auto header = peekHeader( payload );
    if( !header.has_value() )
    {
      return;
    }

    switch( header->msgId )
    {
      case MsgId_MSG_PING_CMD:
        sendPayload( ackFor( *header ) );
        break;

      case MsgId_MSG_PARAM_IO:
      {
        const auto message = decodeMessage<ParamIOMessage>( payload, ParamIOMessage_fields );
        if( !message.has_value() )
        {
          break;
        }

        if( header->subId == ParamIOSubId_GET )
        {
          const int paramId = static_cast<int>( message->id );
          const ParamType type = m_parameterTypes.value( paramId, ParamType_UINT32 );
          bool encoded         = false;
          const QVariant value = m_parameterValues.value( paramId, 0 );
          const QByteArray data = encodeParameterValue( type, value, &encoded );
          if( encoded )
          {
            sendPayload( parameterReply( header->uuid, message->id, type, data ) );
          }
          else
          {
            sendPayload( nackFor( *header, StatusCode_INVALID_TYPE ) );
          }
        }
        else if( header->subId == ParamIOSubId_SET )
        {
          const int paramId = static_cast<int>( message->id );
          const QByteArray data( reinterpret_cast<const char *>( message->data.bytes ),
                                 static_cast<int>( message->data.size ) );
          bool decoded        = false;
          const QVariant value = decodeParameterValue( message->type, data, &decoded );
          if( decoded )
          {
            m_parameterTypes.insert( paramId, message->type );
            m_parameterValues.insert( paramId, value );
            sendPayload( ackFor( *header ) );
          }
          else
          {
            sendPayload( nackFor( *header, StatusCode_INVALID_VALUE ) );
          }
        }
        break;
      }

      case MsgId_MSG_SYS_CTRL:
        if( header->subId == SystemControlSubId_ENABLE_STREAM_PHASE_CURRENTS )
        {
          setStreamEnabled( true );
        }
        else if( header->subId == SystemControlSubId_DISABLE_STREAM_PHASE_CURRENTS )
        {
          setStreamEnabled( false );
        }
        sendPayload( ackFor( *header ) );
        break;

      default:
        sendPayload( ackFor( *header ) );
        break;
    }
  }

  void FakeDeviceServer::sendPayload( const QByteArray &payload )
  {
    if( m_peer && !payload.isEmpty() )
    {
      m_peer->write( encodeFrame( payload ) );
    }
  }

  void FakeDeviceServer::sendTick()
  {
    sendPayload( tickMessage( m_tick++ ) );
  }

  void FakeDeviceServer::sendStatus()
  {
    sendPayload( statusMessage( m_motorState, m_tick ) );
  }

  void FakeDeviceServer::sendStreamSample()
  {
    sendPayload( phaseCurrentsMessage( 1.0f, -0.5f, 0.25f, m_sampleTimestamp ) );
    m_sampleTimestamp += 33333;
  }

  void FakeDeviceServer::stopTimers()
  {
    m_tickTimer.stop();
    m_statusTimer.stop();
    m_streamTimer.stop();
  }
}    // namespace orbit::test
