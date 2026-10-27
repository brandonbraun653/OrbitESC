#include "device/DeviceClient.h"

#include "device/TransportRegistry.h"

#include <QDateTime>

namespace orbit::device
{
  namespace
  {
    constexpr int TickTimeoutMs    = 3000;
    constexpr int ParameterTimeout = 2000;
    constexpr int ControlTimeout   = 2000;
    constexpr int ResetTimeout     = 3000;
    constexpr int PingTimeout      = 2000;
  }

  DeviceClient::DeviceClient( TransportRegistry *registry, QObject *parent ) :
      QObject( parent ),
      m_registry( registry )
  {
    initialize();
  }

  DeviceClient::DeviceClient( Transport *transport, QObject *parent ) :
      QObject( parent ),
      m_transport( transport ),
      m_ownsTransport( false )
  {
    initialize();
    wireTransport();
  }

  DeviceClient::~DeviceClient()
  {
    if( m_ownsTransport && m_transport )
    {
      delete m_transport;
      m_transport = nullptr;
    }
  }

  void DeviceClient::initialize()
  {
    qRegisterMetaType<ConnectionTarget>();
    qRegisterMetaType<DeviceClient::State>();
    qRegisterMetaType<DeviceClient::Command>();

    m_requestTimer = new QTimer( this );
    m_tickTimer = new QTimer( this );
    m_requestTimer->setSingleShot( true );
    m_tickTimer->setSingleShot( true );
    connect( m_requestTimer, &QTimer::timeout, this, &DeviceClient::handleRequestTimeout );
    connect( m_tickTimer, &QTimer::timeout, this, &DeviceClient::handleTickTimeout );
  }

  void DeviceClient::wireTransport()
  {
    if( !m_transport )
    {
      return;
    }

    connect( m_transport, &Transport::opened, this, &DeviceClient::handleTransportOpened );
    connect( m_transport, &Transport::closed, this, &DeviceClient::handleTransportClosed );
    connect( m_transport, &Transport::dataReceived, this, &DeviceClient::handleTransportData );
    connect( m_transport, &Transport::errorOccurred, this, &DeviceClient::handleTransportError );
  }

  void DeviceClient::destroyTransport()
  {
    if( !m_transport )
    {
      return;
    }

    disconnect( m_transport, nullptr, this, nullptr );
    if( m_ownsTransport )
    {
      m_transport->close();
      m_transport->deleteLater();
    }
    m_transport = nullptr;
  }

  void DeviceClient::openTarget( const ConnectionTarget &target )
  {
    if( m_state != State::Disconnected )
    {
      return;
    }

    m_accumulator.clear();

    if( m_registry )
    {
      destroyTransport();
      m_transport = m_registry->create( target, this );
      m_ownsTransport = true;
      wireTransport();
    }

    if( !m_transport )
    {
      emit logMessage( static_cast<int>( Severity::Error ), tr( "No transport available for %1" ).arg( target.displayName ) );
      return;
    }

    setState( State::Connecting );
    if( !m_transport->open( target ) )
    {
      setState( State::Disconnected );
      destroyTransport();
    }
  }

  void DeviceClient::closeConnection()
  {
    m_requestTimer->stop();
    m_tickTimer->stop();
    finishCurrent( false, tr( "Disconnected" ) );
    m_queue.clear();
    m_accumulator.clear();
    if( m_transport )
    {
      m_transport->close();
    }
    setState( State::Disconnected );
    destroyTransport();
  }

  void DeviceClient::requestParameterGet( int paramId )
  {
    PendingRequest request;
    request.command   = Command::GetParameter;
    request.uuid      = nextUuid();
    request.paramId   = paramId;
    request.detail    = paramId;
    request.timeoutMs = ParameterTimeout;
    request.payload   = orbit::protocol::encodeParamGet( static_cast<ParamId>( paramId ), request.uuid );
    enqueue( request );
  }

  void DeviceClient::requestParameterSet( int paramId, int paramType, const QByteArray &data )
  {
    PendingRequest request;
    request.command   = Command::SetParameter;
    request.uuid      = nextUuid();
    request.paramId   = paramId;
    request.paramType = paramType;
    request.timeoutMs = ParameterTimeout;
    request.payload   = orbit::protocol::encodeParamSet( static_cast<ParamId>( paramId ),
                                                         static_cast<ParamType>( paramType ), data, request.uuid );
    enqueue( request );
  }

  void DeviceClient::requestSystemControl( int subId, int detail )
  {
    PendingRequest request;
    request.command   = Command::SystemControl;
    request.uuid      = nextUuid();
    request.detail    = detail ? detail : subId;
    request.timeoutMs = ControlTimeout;
    request.payload   = orbit::protocol::encodeSystemControl( static_cast<SystemControlSubId>( subId ), request.uuid );
    enqueue( request );
  }

  void DeviceClient::requestSystemReset()
  {
    PendingRequest request;
    request.command   = Command::Reset;
    request.uuid      = nextUuid();
    request.detail    = SystemControlSubId_RESET;
    request.timeoutMs = ResetTimeout;
    request.payload   = orbit::protocol::encodeSystemControl( SystemControlSubId_RESET, request.uuid );
    enqueue( request );
  }

  void DeviceClient::requestEmergencyStop()
  {
    PendingRequest request;
    request.command   = Command::SystemControl;
    request.uuid      = nextUuid();
    request.detail    = SystemControlSubId_FAULT;
    request.timeoutMs = ControlTimeout;
    request.payload   = orbit::protocol::encodeSystemControl( SystemControlSubId_FAULT, request.uuid );
    enqueue( request );
  }

  void DeviceClient::setPhaseCurrentStream( bool enable )
  {
    const SystemControlSubId subId = enable ? SystemControlSubId_ENABLE_STREAM_PHASE_CURRENTS
                                            : SystemControlSubId_DISABLE_STREAM_PHASE_CURRENTS;
    PendingRequest request;
    request.command   = Command::Stream;
    request.uuid      = nextUuid();
    request.detail    = enable ? 1 : 0;
    request.timeoutMs = ControlTimeout;
    request.payload   = orbit::protocol::encodeSystemControl( subId, request.uuid );
    enqueue( request );
  }

  void DeviceClient::handleTransportOpened()
  {
    setState( State::Connecting );
    sendPing();
  }

  void DeviceClient::handleTransportClosed()
  {
    m_requestTimer->stop();
    m_tickTimer->stop();
    finishCurrent( false, tr( "Device link closed" ) );
    m_queue.clear();
    setState( State::Disconnected );
  }

  void DeviceClient::handleTransportData( const QByteArray &data )
  {
    m_accumulator.append( data );
    const auto frames = m_accumulator.takeFrames();

    if( frames.overflowed )
    {
      emit logMessage( static_cast<int>( Severity::Warning ), tr( "Receive buffer overflowed" ) );
    }
    if( frames.invalidFrames > 0 )
    {
      emit logMessage( static_cast<int>( Severity::Warning ),
                       tr( "Dropped %1 invalid COBS frame(s)" ).arg( frames.invalidFrames ) );
    }

    for( const QByteArray &frame : frames.payloads )
    {
      handleFrame( frame );
    }

    for( const QString &message : m_consoleAssembler.expire( QDateTime::currentMSecsSinceEpoch() ) )
    {
      emit logMessage( static_cast<int>( Severity::Warning ), message );
    }
  }

  void DeviceClient::handleTransportError( const QString &message )
  {
    emit logMessage( static_cast<int>( Severity::Error ), message );
  }

  void DeviceClient::handleTickTimeout()
  {
    if( m_state == State::Online )
    {
      setState( State::Offline );
      emit logMessage( static_cast<int>( Severity::Warning ), tr( "Link offline: no system tick for 3 seconds" ) );
    }
  }

  void DeviceClient::handleRequestTimeout()
  {
    const bool pingTimedOut = m_inFlight.has_value() && m_inFlight->command == Command::Ping;
    finishCurrent( false, tr( "Request timed out" ) );
    if( pingTimedOut && m_state != State::Online )
    {
      setState( State::Offline );
    }
    sendNextRequest();
  }

  void DeviceClient::setState( State state )
  {
    if( m_state == state )
    {
      return;
    }
    m_state = state;
    emit stateChanged( state );
  }

  void DeviceClient::enqueue( const PendingRequest &request, bool allowWhileNotOnline )
  {
    if( request.payload.isEmpty() )
    {
      emit commandFinished( request.command, request.detail, false, tr( "Failed to encode request" ) );
      return;
    }

    if( m_state != State::Online && !allowWhileNotOnline )
    {
      emit commandFinished( request.command, request.detail, false, tr( "Device is not online" ) );
      return;
    }

    m_queue.enqueue( request );
    sendNextRequest();
  }

  void DeviceClient::sendNextRequest()
  {
    if( m_inFlight.has_value() || m_queue.isEmpty() )
    {
      return;
    }

    m_inFlight = m_queue.dequeue();
    const QByteArray frame = orbit::protocol::encodeFrame( m_inFlight->payload );
    if( frame.isEmpty() )
    {
      finishCurrent( false, tr( "Failed to frame request" ) );
      sendNextRequest();
      return;
    }

    m_transport->write( frame );
    m_requestTimer->start( m_inFlight->timeoutMs );
  }

  void DeviceClient::finishCurrent( bool ok, const QString &error )
  {
    if( !m_inFlight.has_value() )
    {
      return;
    }

    const PendingRequest request = *m_inFlight;
    m_requestTimer->stop();
    m_inFlight.reset();

    if( request.command == Command::SetParameter )
    {
      emit parameterWriteFinished( request.paramId, ok, error );
    }
    emit commandFinished( request.command, request.detail, ok, error );
  }

  void DeviceClient::handleFrame( const QByteArray &frame )
  {
    const auto header = orbit::protocol::peekHeader( frame );
    if( !header.has_value() )
    {
      emit logMessage( static_cast<int>( Severity::Warning ), tr( "Received frame with invalid header" ) );
      return;
    }

    const bool matchesCurrent = m_inFlight.has_value() && header->uuid == m_inFlight->uuid && header->uuid != 0;

    switch( header->msgId )
    {
      case MsgId_MSG_ACK_NACK:
      {
        const auto message = orbit::protocol::decodeMessage<AckNackMessage>( frame, AckNackMessage_fields );
        if( !message.has_value() )
        {
          break;
        }
        if( matchesCurrent )
        {
          const bool acknowledged = message->acknowledge;
          finishCurrent( acknowledged,
                         acknowledged ? QString{}
                                      : tr( "NACK: %1" ).arg( orbit::protocol::statusCodeName( message->status_code ) ) );
          if( !acknowledged && m_state == State::Connecting )
          {
            setState( State::Offline );
          }
          sendNextRequest();
        }
        break;
      }

      case MsgId_MSG_SYS_TICK:
      {
        const auto message = orbit::protocol::decodeMessage<SystemTickMessage>( frame, SystemTickMessage_fields );
        if( message.has_value() )
        {
          noteActivity();
          if( m_state != State::Online )
          {
            setState( State::Online );
            emit logMessage( static_cast<int>( Severity::Info ), tr( "OrbitESC link online" ) );
          }
          emit linkTick( message->tick );
        }
        break;
      }

      case MsgId_MSG_SYS_STATUS:
      {
        const auto message = orbit::protocol::decodeMessage<SystemStatusMessage>( frame, SystemStatusMessage_fields );
        if( message.has_value() )
        {
          noteActivity();
          emit systemStatusChanged( static_cast<int>( message->motorCtrlState ), message->systemTick );
        }
        break;
      }

      case MsgId_MSG_SYS_DATA:
      {
        const auto message = orbit::protocol::decodeMessage<SystemDataMessage>( frame, SystemDataMessage_fields );
        if( !message.has_value() )
        {
          break;
        }

        if( message->id == SystemDataId_ADC_SYSTEM_VOLTAGES )
        {
          const auto voltages = orbit::protocol::decodeSystemVoltages( *message );
          if( voltages.has_value() )
          {
            emit systemVoltagesReceived( voltages->vMcu, voltages->vDcLink, voltages->vTemp, voltages->vIsense );
          }
        }
        else if( message->id == SystemDataId_ADC_PHASE_CURRENTS )
        {
          const auto currents = orbit::protocol::decodePhaseCurrents( *message );
          if( currents.has_value() )
          {
            emit phaseCurrentsReceived( currents->ia, currents->ib, currents->ic, message->timestamp );
          }
        }
        break;
      }

      case MsgId_MSG_PARAM_IO:
      {
        const auto message = orbit::protocol::decodeMessage<ParamIOMessage>( frame, ParamIOMessage_fields );
        if( !message.has_value() )
        {
          break;
        }

        if( matchesCurrent && message->has_type && message->has_data )
        {
          const int responseParamId = m_inFlight->paramId >= 0 ? m_inFlight->paramId : message->id;
          emit parameterValueReceived( responseParamId, message->type,
                                       QByteArray( reinterpret_cast<const char *>( message->data.bytes ),
                                                   static_cast<int>( message->data.size ) ) );
          finishCurrent( true, {} );
          sendNextRequest();
        }
        break;
      }

      case MsgId_MSG_TERMINAL:
      {
        const auto message = orbit::protocol::decodeMessage<ConsoleMessage>( frame, ConsoleMessage_fields );
        if( message.has_value() )
        {
          const auto line = m_consoleAssembler.add( *message );
          if( line.has_value() )
          {
            emit consoleLineReceived( *line );
          }
        }
        break;
      }

      default:
        break;
    }
  }

  void DeviceClient::noteActivity()
  {
    m_tickTimer->start( TickTimeoutMs );
  }

  void DeviceClient::sendPing()
  {
    PendingRequest request;
    request.command   = Command::Ping;
    request.uuid      = nextUuid();
    request.timeoutMs = PingTimeout;
    request.payload   = orbit::protocol::encodePing( request.uuid );
    enqueue( request, true );
  }

  quint16 DeviceClient::nextUuid()
  {
    m_nextUuid++;
    if( m_nextUuid == 0 )
    {
      m_nextUuid = 1;
    }
    return m_nextUuid;
  }
}    // namespace orbit::device
