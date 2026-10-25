#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QQueue>
#include <QString>
#include <QThread>
#include <QTimer>
#include <QUrl>
#include <QVariantList>

#include "device/DeviceClient.h"
#include "model/LogModel.h"
#include "model/ParameterModel.h"
#include "model/TelemetryBuffer.h"

namespace orbit::app
{
  class AppController : public QObject
  {
    Q_OBJECT

    Q_PROPERTY( QVariantList availablePorts READ availablePorts NOTIFY availablePortsChanged )
    Q_PROPERTY( QString selectedPort READ selectedPort NOTIFY selectedPortChanged )
    Q_PROPERTY( QString connectionState READ connectionState NOTIFY connectionStateChanged )
    Q_PROPERTY( bool connected READ connected NOTIFY connectionStateChanged )
    Q_PROPERTY( bool online READ online NOTIFY connectionStateChanged )
    Q_PROPERTY( QString systemMode READ systemMode NOTIFY systemModeChanged )
    Q_PROPERTY( int tickAgeMs READ tickAgeMs NOTIFY tickAgeChanged )
    Q_PROPERTY( double busVoltage READ busVoltage NOTIFY telemetryValuesChanged )
    Q_PROPERTY( double tempSensorVoltage READ tempSensorVoltage NOTIFY telemetryValuesChanged )
    Q_PROPERTY( double phaseCurrentRms READ phaseCurrentRms NOTIFY telemetryValuesChanged )
    Q_PROPERTY( double phaseCurrentPeak READ phaseCurrentPeak NOTIFY telemetryValuesChanged )
    Q_PROPERTY( bool streamEnabled READ streamEnabled NOTIFY streamEnabledChanged )
    Q_PROPERTY( bool busy READ busy NOTIFY busyChanged )
    Q_PROPERTY( QString progressText READ progressText NOTIFY progressTextChanged )
    Q_PROPERTY( int dirtyCount READ dirtyCount NOTIFY dirtyCountChanged )
    Q_PROPERTY( int selectedPage READ selectedPage WRITE setSelectedPage NOTIFY selectedPageChanged )
    Q_PROPERTY( bool plotAutoScale READ plotAutoScale WRITE setPlotAutoScale NOTIFY plotAutoScaleChanged )
    Q_PROPERTY( orbit::model::ParameterModel *informationModel READ informationModel CONSTANT )
    Q_PROPERTY( orbit::model::ParameterModel *systemModel READ systemModel CONSTANT )
    Q_PROPERTY( orbit::model::ParameterModel *motorControlModel READ motorControlModel CONSTANT )
    Q_PROPERTY( orbit::model::ParameterModel *motorAttributesModel READ motorAttributesModel CONSTANT )
    Q_PROPERTY( orbit::model::TelemetryBuffer *telemetry READ telemetry CONSTANT )
    Q_PROPERTY( orbit::model::LogModel *logModel READ logModel CONSTANT )

  public:
    explicit AppController( QObject *parent = nullptr );
    ~AppController() override;

    QVariantList availablePorts() const
    {
      return m_availablePorts;
    }
    QString selectedPort() const
    {
      return m_selectedPort;
    }
    QString connectionState() const;
    bool connected() const;
    bool online() const;
    QString systemMode() const
    {
      return m_systemMode;
    }
    int tickAgeMs() const;
    double busVoltage() const
    {
      return m_busVoltage;
    }
    double tempSensorVoltage() const
    {
      return m_tempSensorVoltage;
    }
    double phaseCurrentRms() const
    {
      return m_phaseCurrentRms;
    }
    double phaseCurrentPeak() const
    {
      return m_phaseCurrentPeak;
    }
    bool streamEnabled() const
    {
      return m_streamEnabled;
    }
    bool busy() const;
    QString progressText() const
    {
      return m_progressText;
    }
    int dirtyCount() const;
    int selectedPage() const
    {
      return m_selectedPage;
    }
    bool plotAutoScale() const
    {
      return m_plotAutoScale;
    }

    orbit::model::ParameterModel *informationModel() const
    {
      return m_informationModel;
    }
    orbit::model::ParameterModel *systemModel() const
    {
      return m_systemModel;
    }
    orbit::model::ParameterModel *motorControlModel() const
    {
      return m_motorControlModel;
    }
    orbit::model::ParameterModel *motorAttributesModel() const
    {
      return m_motorAttributesModel;
    }
    orbit::model::TelemetryBuffer *telemetry() const
    {
      return m_telemetry;
    }
    orbit::model::LogModel *logModel() const
    {
      return m_logModel;
    }

    Q_INVOKABLE void refreshPorts();
    Q_INVOKABLE void selectPort( const QString &portName );
    Q_INVOKABLE void toggleConnection();
    Q_INVOKABLE void connectDevice();
    Q_INVOKABLE void disconnectDevice();
    Q_INVOKABLE void resetSystem();
    Q_INVOKABLE void emergencyStop();
    Q_INVOKABLE void setPhaseCurrentStream( bool enable );
    Q_INVOKABLE void clearTelemetry();
    Q_INVOKABLE void setParameterValue( int paramId, const QVariant &value );
    Q_INVOKABLE void applyParameters();
    Q_INVOKABLE void refreshParameters();
    Q_INVOKABLE void saveParameters( const QUrl &fileUrl );
    Q_INVOKABLE void loadParameters( const QUrl &fileUrl );
    Q_INVOKABLE void clearLog();
    Q_INVOKABLE void logInfo( const QString &message );
    Q_INVOKABLE void saveWindowGeometry( int x, int y, int width, int height );
    Q_INVOKABLE QVariantMap windowGeometry() const;
    void setSelectedPage( int page );
    void setPlotAutoScale( bool enabled );

  signals:
    void availablePortsChanged();
    void selectedPortChanged();
    void connectionStateChanged();
    void systemModeChanged();
    void tickAgeChanged();
    void telemetryValuesChanged();
    void streamEnabledChanged();
    void busyChanged();
    void progressTextChanged();
    void dirtyCountChanged();
    void selectedPageChanged();
    void plotAutoScaleChanged();
    void notification( int severity, const QString &message );

  private slots:
    void handleStateChanged( orbit::device::DeviceClient::State state );
    void handleLinkTick( quint32 tick );
    void handleSystemStatus( int motorCtrlState, quint32 tick );
    void handleSystemVoltages( float vMcu, float vDcLink, float vTemp, float vIsense );
    void handlePhaseCurrents( float ia, float ib, float ic, quint32 timestampUs );
    void handleConsoleLine( const QString &line );
    void handleParameterValue( int paramId, int paramType, const QByteArray &data );
    void handleParameterWriteFinished( int paramId, bool ok, const QString &error );
    void handleCommandFinished( orbit::device::DeviceClient::Command command, int detail, bool ok,
                                const QString &error );
    void handleClientLog( int severity, const QString &message );
    void updateTickAge();
    void updateCurrentStats();

  private:
    enum class Notification
    {
      Info,
      Warning,
      Error
    };

    void setSelectedPort( const QString &portName );
    void setSystemMode( const QString &mode );
    void setBusy( bool busy );
    void setProgress( const QString &text );
    void notify( Notification severity, const QString &message );
    void refreshDirtyCount();
    void sendNextApply();
    void sendNextRefresh();
    void finishApply( bool ok, const QString &message );
    void finishRefresh( bool ok, const QString &message );
    QString modeName( int mode ) const;
    orbit::model::ParameterModel *modelForParamId( int paramId ) const;

    QThread *m_worker = nullptr;
    orbit::device::DeviceClient *m_client = nullptr;

    QVariantList m_availablePorts;
    QString m_selectedPort;
    orbit::device::DeviceClient::State m_state = orbit::device::DeviceClient::State::Disconnected;
    QString m_systemMode = "N/A";
    int m_lastMotorState = -1;
    double m_busVoltage = -1.0;
    double m_tempSensorVoltage = -1.0;
    double m_phaseCurrentRms = -1.0;
    double m_phaseCurrentPeak = -1.0;
    bool m_streamEnabled = false;
    QString m_progressText;
    bool m_hasAutoRefreshed = false;
    int m_selectedPage = 0;
    bool m_plotAutoScale = true;

    QElapsedTimer m_lastTick;
    QTimer m_tickAgeTimer;
    QTimer m_statsTimer;

    QQueue<int> m_applyQueue;
    QQueue<int> m_refreshQueue;
    bool m_applyActive = false;
    bool m_refreshActive = false;
    int m_applyTotal = 0;
    int m_applyDone = 0;
    int m_refreshTotal = 0;
    int m_refreshDone = 0;

    orbit::model::ParameterModel *m_informationModel = nullptr;
    orbit::model::ParameterModel *m_systemModel = nullptr;
    orbit::model::ParameterModel *m_motorControlModel = nullptr;
    orbit::model::ParameterModel *m_motorAttributesModel = nullptr;
    orbit::model::TelemetryBuffer *m_telemetry = nullptr;
    orbit::model::LogModel *m_logModel = nullptr;
  };
}    // namespace orbit::app
