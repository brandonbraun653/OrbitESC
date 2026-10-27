#pragma once

#include <QByteArray>
#include <QObject>
#include <QQueue>
#include <QString>
#include <QTimer>
#include <optional>

#include "protocol/ConsoleAssembler.h"
#include "protocol/FrameCodec.h"
#include "protocol/MessageCodec.h"
#include "device/ConnectionTarget.h"
#include "device/Transport.h"

namespace orbit::device
{
  class TransportRegistry;

  class DeviceClient : public QObject
  {
    Q_OBJECT
  public:
    enum class State
    {
      Disconnected,
      Connecting,
      Online,
      Offline
    };
    Q_ENUM( State )

    enum class Command
    {
      Ping,
      GetParameter,
      SetParameter,
      SystemControl,
      Reset,
      Stream
    };
    Q_ENUM( Command )

    enum class Severity
    {
      Debug,
      Info,
      Warning,
      Error
    };
    Q_ENUM( Severity )

    explicit DeviceClient( TransportRegistry *registry, QObject *parent = nullptr );
    explicit DeviceClient( Transport *transport, QObject *parent = nullptr );
    ~DeviceClient() override;

    State state() const
    {
      return m_state;
    }

  public slots:
    void openTarget( const ConnectionTarget &target );
    void closeConnection();
    void requestParameterGet( int paramId );
    void requestParameterSet( int paramId, int paramType, const QByteArray &data );
    void requestSystemControl( int subId, int detail );
    void requestSystemReset();
    void requestEmergencyStop();
    void setPhaseCurrentStream( bool enable );

  signals:
    void stateChanged( orbit::device::DeviceClient::State state );
    void linkTick( quint32 tick );
    void systemStatusChanged( int motorCtrlState, quint32 tick );
    void systemVoltagesReceived( float vMcu, float vDcLink, float vTemp, float vIsense );
    void phaseCurrentsReceived( float ia, float ib, float ic, quint32 timestampUs );
    void consoleLineReceived( const QString &line );
    void parameterValueReceived( int paramId, int paramType, const QByteArray &data );
    void parameterWriteFinished( int paramId, bool ok, const QString &error );
    void commandFinished( orbit::device::DeviceClient::Command command, int detail, bool ok, const QString &error );
    void logMessage( int severity, const QString &message );

  private slots:
    void handleTransportOpened();
    void handleTransportClosed();
    void handleTransportData( const QByteArray &data );
    void handleTransportError( const QString &message );
    void handleTickTimeout();
    void handleRequestTimeout();

  private:
    struct PendingRequest
    {
      Command command = Command::Ping;
      quint16 uuid    = 0;
      QByteArray payload;
      int paramId     = -1;
      int paramType   = 0;
      int detail      = 0;
      int timeoutMs   = 2000;
    };

    void setState( State state );
    void initialize();
    void wireTransport();
    void destroyTransport();
    void enqueue( const PendingRequest &request, bool allowWhileNotOnline = false );
    void sendNextRequest();
    void finishCurrent( bool ok, const QString &error );
    void handleFrame( const QByteArray &frame );
    void noteActivity();
    void sendPing();
    quint16 nextUuid();

    Transport *m_transport = nullptr;
    TransportRegistry *m_registry = nullptr;
    bool m_ownsTransport = false;
    orbit::protocol::FrameAccumulator m_accumulator;
    orbit::protocol::ConsoleAssembler m_consoleAssembler;
    QTimer *m_requestTimer = nullptr;
    QTimer *m_tickTimer = nullptr;
    QQueue<PendingRequest> m_queue;
    std::optional<PendingRequest> m_inFlight;
    State m_state = State::Disconnected;
    quint16 m_nextUuid = 0;
  };
}    // namespace orbit::device

Q_DECLARE_METATYPE( orbit::device::DeviceClient::State )
Q_DECLARE_METATYPE( orbit::device::DeviceClient::Command )
