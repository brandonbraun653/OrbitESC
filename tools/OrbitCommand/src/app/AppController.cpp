#include "app/AppController.h"

#include "device/SimulatorDefaults.h"
#include "model/ParameterRegistry.h"
#include "protocol/ParameterCodec.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QSaveFile>

namespace orbit::app
{
  namespace
  {
    constexpr int TickAgeRefreshMs = 500;
    constexpr int StatsRefreshMs   = 200;
    const char *LastTargetKey      = "lastTargetId";
    const char *LegacyLastPortKey  = "lastPort";
    const char *SimulatorHostKey   = "simulatorHost";
    const char *SimulatorPortKey   = "simulatorPort";

    QJsonValue valueToJson( ParamType type, const QVariant &value )
    {
      switch( type )
      {
        case ParamType_BOOL:
          return value.toBool();
        case ParamType_FLOAT:
        case ParamType_DOUBLE:
          return value.toDouble();
        case ParamType_STRING:
          return value.toString();
        case ParamType_BYTES:
          return QString::fromLatin1( value.toByteArray().toBase64() );
        default:
          return value.toLongLong();
      }
    }

    QVariant jsonToValue( ParamType type, const QJsonValue &value )
    {
      switch( type )
      {
        case ParamType_BOOL:
          return value.toBool();
        case ParamType_FLOAT:
        case ParamType_DOUBLE:
          return value.toDouble();
        case ParamType_STRING:
          return value.toString();
        case ParamType_BYTES:
          return QByteArray::fromBase64( value.toString().toLatin1() );
        default:
          return static_cast<qlonglong>( value.toDouble() );
      }
    }
  }    // namespace

  AppController::AppController( QObject *parent ) : QObject( parent )
  {
    m_informationModel = new orbit::model::ParameterModel(
        orbit::model::registry::forGroup( orbit::model::ParameterGroup::Information ), this );
    m_systemModel = new orbit::model::ParameterModel(
        orbit::model::registry::forGroup( orbit::model::ParameterGroup::SystemBehavior ), this );
    m_motorControlModel = new orbit::model::ParameterModel(
        orbit::model::registry::forGroup( orbit::model::ParameterGroup::MotorControl ), this );
    m_motorAttributesModel = new orbit::model::ParameterModel(
        orbit::model::registry::forGroup( orbit::model::ParameterGroup::MotorAttributes ), this );
    m_telemetry = new orbit::model::TelemetryBuffer( this );
    m_logModel  = new orbit::model::LogModel( this );

    m_worker = new QThread( this );
    m_client = new orbit::device::DeviceClient( &m_transportRegistry );
    m_client->moveToThread( m_worker );

    connect( m_client, &orbit::device::DeviceClient::stateChanged, this, &AppController::handleStateChanged );
    connect( m_client, &orbit::device::DeviceClient::linkTick, this, &AppController::handleLinkTick );
    connect( m_client, &orbit::device::DeviceClient::systemStatusChanged, this, &AppController::handleSystemStatus );
    connect( m_client, &orbit::device::DeviceClient::systemVoltagesReceived, this,
             &AppController::handleSystemVoltages );
    connect( m_client, &orbit::device::DeviceClient::phaseCurrentsReceived, this,
             &AppController::handlePhaseCurrents );
    connect( m_client, &orbit::device::DeviceClient::consoleLineReceived, this, &AppController::handleConsoleLine );
    connect( m_client, &orbit::device::DeviceClient::parameterValueReceived, this,
             &AppController::handleParameterValue );
    connect( m_client, &orbit::device::DeviceClient::parameterWriteFinished, this,
             &AppController::handleParameterWriteFinished );
    connect( m_client, &orbit::device::DeviceClient::commandFinished, this, &AppController::handleCommandFinished );
    connect( m_client, &orbit::device::DeviceClient::logMessage, this, &AppController::handleClientLog );

    m_worker->start();

    m_tickAgeTimer.setInterval( TickAgeRefreshMs );
    m_statsTimer.setInterval( StatsRefreshMs );
    connect( &m_tickAgeTimer, &QTimer::timeout, this, &AppController::updateTickAge );
    connect( &m_statsTimer, &QTimer::timeout, this, &AppController::updateCurrentStats );
    m_tickAgeTimer.start();
    m_statsTimer.start();

    QSettings settings;
    m_selectedTargetId = settings.value( LastTargetKey ).toString();
    if( m_selectedTargetId.isEmpty() )
    {
      const QString legacyPort = settings.value( LegacyLastPortKey ).toString();
      if( !legacyPort.isEmpty() )
      {
        m_selectedTargetId = QStringLiteral( "serial:%1" ).arg( legacyPort );
        settings.setValue( LastTargetKey, m_selectedTargetId );
      }
    }
    m_simulatorHost = settings.value( SimulatorHostKey, QString::fromLatin1( orbit::device::DefaultSimulatorHost ) )
                          .toString()
                          .trimmed();
    if( m_simulatorHost.isEmpty() )
    {
      m_simulatorHost = QString::fromLatin1( orbit::device::DefaultSimulatorHost );
    }
    m_simulatorPort = qBound( 1, settings.value( SimulatorPortKey, orbit::device::DefaultSimulatorPort ).toInt(),
                              65535 );
    m_selectedPage = qBound( 0, settings.value( "selectedPage", 0 ).toInt(), 3 );
    m_plotAutoScale = settings.value( "plotAutoScale", true ).toBool();
    refreshTargets();

    m_logModel->addApp( orbit::model::LogModel::Severity::Info, tr( "OrbitCommand initialized" ) );
  }

  AppController::~AppController()
  {
    if( m_worker && m_worker->isRunning() && m_client )
    {
      QMetaObject::invokeMethod( m_client, "closeConnection", Qt::BlockingQueuedConnection );
      QMetaObject::invokeMethod( m_client, [client = m_client]() { delete client; }, Qt::BlockingQueuedConnection );
      m_client = nullptr;
    }
    else if( m_client )
    {
      delete m_client;
      m_client = nullptr;
    }
    if( m_worker )
    {
      m_worker->quit();
      m_worker->wait();
    }
  }

  QString AppController::connectionState() const
  {
    switch( m_state )
    {
      case orbit::device::DeviceClient::State::Connecting:
        return "Connecting";
      case orbit::device::DeviceClient::State::Online:
        return "Online";
      case orbit::device::DeviceClient::State::Offline:
        return "Offline";
      case orbit::device::DeviceClient::State::Disconnected:
      default:
        return "Disconnected";
    }
  }

  bool AppController::connected() const
  {
    return m_state != orbit::device::DeviceClient::State::Disconnected;
  }

  bool AppController::online() const
  {
    return m_state == orbit::device::DeviceClient::State::Online;
  }

  int AppController::tickAgeMs() const
  {
    if( m_state != orbit::device::DeviceClient::State::Online || !m_lastTick.isValid() )
    {
      return -1;
    }
    return static_cast<int>( m_lastTick.elapsed() );
  }

  bool AppController::busy() const
  {
    return m_applyActive || m_refreshActive;
  }

  int AppController::dirtyCount() const
  {
    return m_informationModel->dirtyCount() + m_systemModel->dirtyCount() + m_motorControlModel->dirtyCount() +
           m_motorAttributesModel->dirtyCount();
  }

  void AppController::refreshTargets()
  {
    QVariantList targets;
    const auto discovered = m_transportRegistry.discoverTargets();
    for( const orbit::device::ConnectionTarget &target : discovered )
    {
      QVariantMap entry;
      entry.insert( "id", target.id );
      entry.insert( "kind", target.kind );
      entry.insert( "label", target.displayName );
      for( auto it = target.properties.cbegin(); it != target.properties.cend(); ++it )
      {
        entry.insert( it.key(), it.value() );
      }

      if( target.kind == QStringLiteral( "simulator" ) )
      {
        entry.insert( "host", m_simulatorHost );
        entry.insert( "port", m_simulatorPort );
        entry.insert( "description", tr( "TCP %1:%2" ).arg( m_simulatorHost ).arg( m_simulatorPort ) );
      }

      targets.append( entry );
    }

    m_availableTargets = targets;
    emit availableTargetsChanged();

    bool selectedStillExists = false;
    for( const QVariant &target : m_availableTargets )
    {
      if( target.toMap().value( "id" ).toString() == m_selectedTargetId )
      {
        selectedStillExists = true;
        break;
      }
    }

    if( selectedStillExists )
    {
      return;
    }

    QString fallbackId;
    for( const QVariant &target : m_availableTargets )
    {
      const QVariantMap entry = target.toMap();
      if( entry.value( "kind" ).toString() == QStringLiteral( "serial" ) &&
          entry.value( "likelyOrbit" ).toBool() )
      {
        fallbackId = entry.value( "id" ).toString();
        break;
      }
    }

    if( fallbackId.isEmpty() )
    {
      for( const QVariant &target : m_availableTargets )
      {
        const QVariantMap entry = target.toMap();
        if( entry.value( "kind" ).toString() == QStringLiteral( "serial" ) )
        {
          fallbackId = entry.value( "id" ).toString();
          break;
        }
      }
    }

    if( fallbackId.isEmpty() )
    {
      for( const QVariant &target : m_availableTargets )
      {
        const QVariantMap entry = target.toMap();
        if( entry.value( "kind" ).toString() == QStringLiteral( "simulator" ) )
        {
          fallbackId = entry.value( "id" ).toString();
          break;
        }
      }
    }

    setSelectedTargetId( fallbackId );
  }

  void AppController::selectTarget( const QString &targetId )
  {
    setSelectedTargetId( targetId );
  }

  void AppController::setSelectedTargetId( const QString &targetId )
  {
    if( m_selectedTargetId == targetId )
    {
      return;
    }

    m_selectedTargetId = targetId;
    QSettings settings;
    settings.setValue( LastTargetKey, m_selectedTargetId );
    emit selectedTargetIdChanged();
  }

  void AppController::setSimulatorHost( const QString &host )
  {
    const QString trimmedHost = host.trimmed();
    if( m_simulatorHost == trimmedHost )
    {
      return;
    }

    m_simulatorHost = trimmedHost;
    QSettings settings;
    settings.setValue( SimulatorHostKey, m_simulatorHost );
    emit simulatorHostChanged();
    updateSimulatorTargetEntry();
  }

  void AppController::setSimulatorPort( int port )
  {
    const int boundedPort = qBound( 1, port, 65535 );
    if( m_simulatorPort == boundedPort )
    {
      return;
    }

    m_simulatorPort = boundedPort;
    QSettings settings;
    settings.setValue( SimulatorPortKey, m_simulatorPort );
    emit simulatorPortChanged();
    updateSimulatorTargetEntry();
  }

  void AppController::updateSimulatorTargetEntry()
  {
    for( QVariant &target : m_availableTargets )
    {
      QVariantMap entry = target.toMap();
      if( entry.value( "kind" ).toString() != QStringLiteral( "simulator" ) )
      {
        continue;
      }

      entry.insert( "host", m_simulatorHost );
      entry.insert( "port", m_simulatorPort );
      entry.insert( "description", tr( "TCP %1:%2" ).arg( m_simulatorHost ).arg( m_simulatorPort ) );
      target = entry;
    }
    emit availableTargetsChanged();
  }

  QVariantMap AppController::selectedTargetMap() const
  {
    for( const QVariant &target : m_availableTargets )
    {
      const QVariantMap entry = target.toMap();
      if( entry.value( "id" ).toString() == m_selectedTargetId )
      {
        return entry;
      }
    }
    return {};
  }

  void AppController::toggleConnection()
  {
    if( connected() )
    {
      disconnectDevice();
    }
    else
    {
      connectDevice();
    }
  }

  void AppController::connectDevice()
  {
    if( connected() )
    {
      return;
    }

    const QVariantMap selected = selectedTargetMap();
    if( selected.isEmpty() )
    {
      notify( Notification::Warning, tr( "Select a connection target first" ) );
      return;
    }

    orbit::device::ConnectionTarget target;
    target.kind = selected.value( "kind" ).toString();
    target.id = selected.value( "id" ).toString();
    target.displayName = selected.value( "label" ).toString();
    for( auto it = selected.cbegin(); it != selected.cend(); ++it )
    {
      if( it.key() != QStringLiteral( "id" ) && it.key() != QStringLiteral( "kind" ) &&
          it.key() != QStringLiteral( "label" ) && it.key() != QStringLiteral( "description" ) )
      {
        target.properties.insert( it.key(), it.value() );
      }
    }

    if( target.kind == QStringLiteral( "simulator" ) )
    {
      if( m_simulatorHost.isEmpty() )
      {
        notify( Notification::Warning, tr( "Enter a simulator host" ) );
        return;
      }
      if( m_simulatorPort < 1 || m_simulatorPort > 65535 )
      {
        notify( Notification::Warning, tr( "Enter a simulator port between 1 and 65535" ) );
        return;
      }

      target.properties.insert( "host", m_simulatorHost );
      target.properties.insert( "port", m_simulatorPort );
      setProgress( tr( "Connecting to simulator at %1:%2" ).arg( m_simulatorHost ).arg( m_simulatorPort ) );
    }
    else
    {
      const QString portName = target.properties.value( "portName" ).toString();
      if( portName.isEmpty() )
      {
        notify( Notification::Warning, tr( "Select a serial port first" ) );
        return;
      }
      setProgress( tr( "Opening %1" ).arg( portName ) );
    }

    m_hasAutoRefreshed = false;
    QMetaObject::invokeMethod( m_client, "openTarget", Qt::QueuedConnection,
                               Q_ARG( orbit::device::ConnectionTarget, target ) );
  }

  void AppController::disconnectDevice()
  {
    setProgress( {} );
    QMetaObject::invokeMethod( m_client, "closeConnection", Qt::QueuedConnection );
  }

  void AppController::resetSystem()
  {
    if( !online() )
    {
      notify( Notification::Warning, tr( "Device is not online" ) );
      return;
    }

    m_logModel->addApp( orbit::model::LogModel::Severity::Warning, tr( "System reset requested" ) );
    QMetaObject::invokeMethod( m_client, "requestSystemReset", Qt::QueuedConnection );
  }

  void AppController::emergencyStop()
  {
    if( !online() )
    {
      notify( Notification::Error, tr( "Cannot send E-STOP while offline" ) );
      return;
    }

    m_logModel->addApp( orbit::model::LogModel::Severity::Error, tr( "EMERGENCY STOP sent" ) );
    notify( Notification::Error, tr( "Emergency stop sent" ) );
    QMetaObject::invokeMethod( m_client, "requestEmergencyStop", Qt::QueuedConnection );
  }

  void AppController::setPhaseCurrentStream( bool enable )
  {
    if( !online() )
    {
      notify( Notification::Warning, tr( "Connect to the device before streaming" ) );
      return;
    }
    if( enable == m_streamEnabled )
    {
      return;
    }

    setProgress( enable ? tr( "Enabling phase-current stream" ) : tr( "Disabling phase-current stream" ) );
    QMetaObject::invokeMethod( m_client, "setPhaseCurrentStream", Qt::QueuedConnection, Q_ARG( bool, enable ) );
  }

  void AppController::clearTelemetry()
  {
    m_telemetry->clear();
    m_phaseCurrentRms  = -1.0;
    m_phaseCurrentPeak = -1.0;
    emit telemetryValuesChanged();
  }

  void AppController::setParameterValue( int paramId, const QVariant &value )
  {
    orbit::model::ParameterModel *model = modelForParamId( paramId );
    if( !model )
    {
      return;
    }

    const orbit::model::ParameterDescriptor *descriptor = orbit::model::registry::find( paramId );
    if( descriptor && descriptor->type == ParamType_STRING && value.toString().size() > descriptor->maxStringLength )
    {
      notify( Notification::Warning, tr( "%1 is limited to %2 characters" )
                                         .arg( descriptor->label )
                                         .arg( descriptor->maxStringLength ) );
    }

    QVariant adjusted = value;
    if( descriptor && descriptor->type == ParamType_STRING )
    {
      adjusted = value.toString().left( descriptor->maxStringLength );
    }

    if( model->setValueForParamId( paramId, adjusted, true ) )
    {
      refreshDirtyCount();
    }
  }

  void AppController::applyParameters()
  {
    if( busy() )
    {
      return;
    }
    if( !online() )
    {
      notify( Notification::Warning, tr( "Device is not online" ) );
      return;
    }

    m_applyQueue.clear();
    const auto models = { m_informationModel, m_systemModel, m_motorControlModel, m_motorAttributesModel };
    for( auto *model : models )
    {
      for( int paramId : model->dirtyParamIds() )
      {
        m_applyQueue.enqueue( paramId );
      }
    }

    if( m_applyQueue.isEmpty() )
    {
      notify( Notification::Info, tr( "No parameter changes to apply" ) );
      return;
    }

    m_applyActive = true;
    m_applyTotal  = m_applyQueue.size();
    m_applyDone   = 0;
    setBusy( true );
    sendNextApply();
  }

  void AppController::sendNextApply()
  {
    if( m_applyQueue.isEmpty() )
    {
      finishApply( true, tr( "Parameters applied" ) );
      return;
    }

    const int paramId = m_applyQueue.dequeue();
    orbit::model::ParameterModel *model = modelForParamId( paramId );
    const auto *descriptor = orbit::model::registry::find( paramId );
    if( !model || !descriptor )
    {
      finishApply( false, tr( "Unknown parameter id %1" ).arg( paramId ) );
      return;
    }

    bool encoded = false;
    const QByteArray data = orbit::protocol::encodeParameterValue( descriptor->type, model->valueForParamId( paramId ),
                                                                   &encoded );
    if( !encoded )
    {
      finishApply( false, tr( "Could not encode %1" ).arg( descriptor->label ) );
      return;
    }

    setProgress( tr( "Applying %1 of %2: %3" )
                     .arg( m_applyDone + 1 )
                     .arg( m_applyTotal )
                     .arg( descriptor->label ) );
    QMetaObject::invokeMethod( m_client, "requestParameterSet", Qt::QueuedConnection, Q_ARG( int, paramId ),
                               Q_ARG( int, static_cast<int>( descriptor->type ) ), Q_ARG( QByteArray, data ) );
  }

  void AppController::refreshParameters()
  {
    if( busy() )
    {
      return;
    }
    if( !online() )
    {
      notify( Notification::Warning, tr( "Device is not online" ) );
      return;
    }

    m_refreshQueue.clear();
    const auto models = { m_informationModel, m_systemModel, m_motorControlModel, m_motorAttributesModel };
    for( auto *model : models )
    {
      for( int paramId : model->allParamIds() )
      {
        m_refreshQueue.enqueue( paramId );
      }
    }

    m_refreshActive = true;
    m_refreshTotal  = m_refreshQueue.size();
    m_refreshDone   = 0;
    setBusy( true );
    sendNextRefresh();
  }

  void AppController::sendNextRefresh()
  {
    if( m_refreshQueue.isEmpty() )
    {
      finishRefresh( true, tr( "Parameters refreshed" ) );
      return;
    }

    const int paramId = m_refreshQueue.dequeue();
    const auto *descriptor = orbit::model::registry::find( paramId );
    setProgress( tr( "Reading %1 of %2: %3" )
                     .arg( m_refreshDone + 1 )
                     .arg( m_refreshTotal )
                     .arg( descriptor ? descriptor->label : QString::number( paramId ) ) );
    QMetaObject::invokeMethod( m_client, "requestParameterGet", Qt::QueuedConnection, Q_ARG( int, paramId ) );
  }

  void AppController::finishApply( bool ok, const QString &message )
  {
    m_applyActive = false;
    m_applyQueue.clear();
    setBusy( false );
    setProgress( {} );
    refreshDirtyCount();
    notify( ok ? Notification::Info : Notification::Error, message );
  }

  void AppController::finishRefresh( bool ok, const QString &message )
  {
    m_refreshActive = false;
    m_refreshQueue.clear();
    setBusy( false );
    setProgress( {} );
    notify( ok ? Notification::Info : Notification::Warning, message );
  }

  void AppController::saveParameters( const QUrl &fileUrl )
  {
    const QString path = fileUrl.isLocalFile() ? fileUrl.toLocalFile() : fileUrl.toString();
    if( path.isEmpty() )
    {
      notify( Notification::Error, tr( "No file selected" ) );
      return;
    }

    QJsonArray parameters;
    const auto models = { m_informationModel, m_systemModel, m_motorControlModel, m_motorAttributesModel };
    for( auto *model : models )
    {
      for( const auto &descriptor : model->descriptors() )
      {
        QJsonObject entry;
        entry.insert( "id", descriptor.id );
        entry.insert( "key", descriptor.key );
        entry.insert( "type", orbit::protocol::parameterTypeName( descriptor.type ) );
        entry.insert( "value", valueToJson( descriptor.type, model->valueForParamId( descriptor.id ) ) );
        parameters.append( entry );
      }
    }

    QJsonObject root;
    root.insert( "schema", "orbitcommand.parameters" );
    root.insert( "version", 1 );
    root.insert( "parameters", parameters );

    QSaveFile file( path );
    if( !file.open( QIODevice::WriteOnly ) )
    {
      notify( Notification::Error, tr( "Could not write %1" ).arg( path ) );
      return;
    }

    file.write( QJsonDocument( root ).toJson( QJsonDocument::Indented ) );
    if( !file.commit() )
    {
      notify( Notification::Error, tr( "Could not commit %1" ).arg( path ) );
      return;
    }

    notify( Notification::Info, tr( "Saved %1 parameters to %2" ).arg( parameters.size() ).arg( path ) );
  }

  void AppController::loadParameters( const QUrl &fileUrl )
  {
    const QString path = fileUrl.isLocalFile() ? fileUrl.toLocalFile() : fileUrl.toString();
    if( path.isEmpty() )
    {
      notify( Notification::Error, tr( "No file selected" ) );
      return;
    }

    QFile file( path );
    if( !file.open( QIODevice::ReadOnly ) )
    {
      notify( Notification::Error, tr( "Could not open %1" ).arg( path ) );
      return;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson( file.readAll(), &parseError );
    if( parseError.error != QJsonParseError::NoError || !document.isObject() )
    {
      notify( Notification::Error, tr( "Invalid JSON: %1" ).arg( parseError.errorString() ) );
      return;
    }

    const QJsonObject root = document.object();
    if( root.value( "schema" ).toString() != "orbitcommand.parameters" || root.value( "version" ).toInt() != 1 )
    {
      notify( Notification::Error, tr( "Unsupported parameter file format" ) );
      return;
    }

    int loaded = 0;
    QStringList issues;
    const QJsonArray parameters = root.value( "parameters" ).toArray();
    for( const QJsonValue &value : parameters )
    {
      const QJsonObject entry = value.toObject();
      const int id = entry.value( "id" ).toInt( -1 );
      const auto *descriptor = orbit::model::registry::find( id );
      if( !descriptor )
      {
        issues.append( tr( "Unknown parameter id %1" ).arg( id ) );
        continue;
      }
      if( entry.value( "type" ).toString() != orbit::protocol::parameterTypeName( descriptor->type ) )
      {
        issues.append( tr( "Type mismatch for %1" ).arg( descriptor->label ) );
        continue;
      }
      if( descriptor->readOnly )
      {
        continue;
      }

      orbit::model::ParameterModel *model = modelForParamId( id );
      if( !model )
      {
        continue;
      }

      model->setValueForParamId( id, jsonToValue( descriptor->type, entry.value( "value" ) ), true );
      loaded++;
    }

    refreshDirtyCount();
    const QString message = issues.isEmpty()
                                ? tr( "Loaded %1 parameters. Review and Apply." ).arg( loaded )
                                : tr( "Loaded %1 parameters with %2 issue(s). Review and Apply." )
                                      .arg( loaded )
                                      .arg( issues.size() );
    notify( issues.isEmpty() ? Notification::Info : Notification::Warning, message );
    for( const QString &issue : issues )
    {
      m_logModel->addApp( orbit::model::LogModel::Severity::Warning, issue );
    }
  }

  void AppController::clearLog()
  {
    m_logModel->clear();
  }

  void AppController::logInfo( const QString &message )
  {
    m_logModel->addApp( orbit::model::LogModel::Severity::Info, message );
  }

  void AppController::saveWindowGeometry( int x, int y, int width, int height )
  {
    QSettings settings;
    settings.setValue( "windowX", x );
    settings.setValue( "windowY", y );
    settings.setValue( "windowWidth", width );
    settings.setValue( "windowHeight", height );
  }

  QVariantMap AppController::windowGeometry() const
  {
    QSettings settings;
    QVariantMap geometry;
    geometry.insert( "valid", settings.contains( "windowWidth" ) && settings.contains( "windowHeight" ) );
    geometry.insert( "x", settings.value( "windowX", 0 ).toInt() );
    geometry.insert( "y", settings.value( "windowY", 0 ).toInt() );
    geometry.insert( "width", settings.value( "windowWidth", 1440 ).toInt() );
    geometry.insert( "height", settings.value( "windowHeight", 900 ).toInt() );
    return geometry;
  }

  void AppController::setSelectedPage( int page )
  {
    page = qBound( 0, page, 3 );
    if( m_selectedPage == page )
    {
      return;
    }
    m_selectedPage = page;
    QSettings settings;
    settings.setValue( "selectedPage", m_selectedPage );
    emit selectedPageChanged();
  }

  void AppController::setPlotAutoScale( bool enabled )
  {
    if( m_plotAutoScale == enabled )
    {
      return;
    }
    m_plotAutoScale = enabled;
    QSettings settings;
    settings.setValue( "plotAutoScale", m_plotAutoScale );
    emit plotAutoScaleChanged();
  }

  void AppController::handleStateChanged( orbit::device::DeviceClient::State state )
  {
    m_state = state;
    emit connectionStateChanged();

    if( state == orbit::device::DeviceClient::State::Online )
    {
      m_lastTick.start();
      if( !m_hasAutoRefreshed )
      {
        m_hasAutoRefreshed = true;
        refreshParameters();
      }
    }
    else if( state == orbit::device::DeviceClient::State::Disconnected )
    {
      m_hasAutoRefreshed = false;
      m_lastTick.invalidate();
      m_streamEnabled = false;
      emit streamEnabledChanged();
      m_systemMode = "N/A";
      emit systemModeChanged();
      m_busVoltage = -1.0;
      m_tempSensorVoltage = -1.0;
      m_phaseCurrentRms = -1.0;
      m_phaseCurrentPeak = -1.0;
      emit telemetryValuesChanged();
      m_applyActive = false;
      m_refreshActive = false;
      m_applyQueue.clear();
      m_refreshQueue.clear();
      setBusy( false );
      setProgress( {} );
    }
    else
    {
      m_lastTick.invalidate();
    }
    emit tickAgeChanged();
  }

  void AppController::handleLinkTick( quint32 tick )
  {
    Q_UNUSED( tick )
    m_lastTick.start();
  }

  void AppController::handleSystemStatus( int motorCtrlState, quint32 tick )
  {
    Q_UNUSED( tick )
    if( motorCtrlState != m_lastMotorState )
    {
      m_lastMotorState = motorCtrlState;
      setSystemMode( modeName( motorCtrlState ) );
    }
  }

  void AppController::handleSystemVoltages( float vMcu, float vDcLink, float vTemp, float vIsense )
  {
    Q_UNUSED( vMcu )
    Q_UNUSED( vIsense )
    m_busVoltage = vDcLink;
    m_tempSensorVoltage = vTemp;
    emit telemetryValuesChanged();
  }

  void AppController::handlePhaseCurrents( float ia, float ib, float ic, quint32 timestampUs )
  {
    m_telemetry->append( timestampUs, ia, ib, ic );
  }

  void AppController::handleConsoleLine( const QString &line )
  {
    m_logModel->addDevice( line );
  }

  void AppController::handleParameterValue( int paramId, int paramType, const QByteArray &data )
  {
    const auto *descriptor = orbit::model::registry::find( paramId );
    orbit::model::ParameterModel *model = modelForParamId( paramId );
    if( !descriptor || !model )
    {
      return;
    }

    bool ok = false;
    const QVariant value = orbit::protocol::decodeParameterValue( static_cast<ParamType>( paramType ), data, &ok );
    if( !ok )
    {
      m_logModel->addApp( orbit::model::LogModel::Severity::Warning,
                          tr( "Could not decode %1" ).arg( descriptor->label ) );
      return;
    }

    model->setDeviceValue( paramId, value );
  }

  void AppController::handleParameterWriteFinished( int paramId, bool ok, const QString &error )
  {
    if( !m_applyActive )
    {
      return;
    }

    if( !ok )
    {
      finishApply( false, tr( "Apply failed for %1: %2" )
                              .arg( orbit::model::registry::find( paramId ) ? orbit::model::registry::find( paramId )->label
                                                                           : QString::number( paramId ),
                                   error ) );
      return;
    }

    if( orbit::model::ParameterModel *model = modelForParamId( paramId ) )
    {
      model->setClean( paramId );
    }
    m_applyDone++;
    refreshDirtyCount();
    sendNextApply();
  }

  void AppController::handleCommandFinished( orbit::device::DeviceClient::Command command, int detail, bool ok,
                                             const QString &error )
  {
    using Command = orbit::device::DeviceClient::Command;

    if( command == Command::GetParameter )
    {
      if( m_refreshActive )
      {
        if( !ok )
        {
          finishRefresh( false, tr( "Refresh failed: %1" ).arg( error ) );
          return;
        }
        m_refreshDone++;
        sendNextRefresh();
      }
      return;
    }

    if( command == Command::SetParameter )
    {
      return;
    }

    if( command == Command::Stream )
    {
      setProgress( {} );
      if( ok )
      {
        m_streamEnabled = detail == 1;
        emit streamEnabledChanged();
        notify( Notification::Info, m_streamEnabled ? tr( "Phase-current stream enabled" )
                                                    : tr( "Phase-current stream disabled" ) );
      }
      else
      {
        notify( Notification::Error, tr( "Stream command failed: %1" ).arg( error ) );
      }
      return;
    }

    if( command == Command::Reset )
    {
      setProgress( {} );
      if( ok )
      {
        notify( Notification::Info, tr( "Reset command accepted" ) );
      }
      else
      {
        notify( Notification::Error, tr( "Reset failed: %1" ).arg( error ) );
      }
      return;
    }

    if( command == Command::SystemControl )
    {
      if( ok )
      {
        if( detail == SystemControlSubId_FAULT )
        {
          m_logModel->addApp( orbit::model::LogModel::Severity::Error, tr( "E-STOP acknowledged" ) );
        }
      }
      else
      {
        notify( Notification::Error, tr( "Control command failed: %1" ).arg( error ) );
      }
      return;
    }

    if( command == Command::Ping && !ok )
    {
      if( !online() )
      {
        notify( Notification::Error, tr( "Device did not answer ping: %1" ).arg( error ) );
      }
      else
      {
        m_logModel->addApp( orbit::model::LogModel::Severity::Debug,
                            tr( "Ping timed out after ticks started" ) );
      }
    }
  }

  void AppController::handleClientLog( int severity, const QString &message )
  {
    const auto logSeverity = static_cast<orbit::model::LogModel::Severity>( qBound( 0, severity, 3 ) );
    m_logModel->addApp( logSeverity, message );
    if( logSeverity == orbit::model::LogModel::Severity::Error )
    {
      emit notification( static_cast<int>( Notification::Error ), message );
    }
  }

  void AppController::updateTickAge()
  {
    emit tickAgeChanged();
  }

  void AppController::updateCurrentStats()
  {
    if( m_telemetry->count() == 0 )
    {
      if( m_phaseCurrentRms >= 0.0 || m_phaseCurrentPeak >= 0.0 )
      {
        m_phaseCurrentRms  = -1.0;
        m_phaseCurrentPeak = -1.0;
        emit telemetryValuesChanged();
      }
      return;
    }

    const double rms = m_telemetry->rmsCurrent();
    const double peak = m_telemetry->peakCurrent();
    if( !qFuzzyCompare( rms + 1.0, m_phaseCurrentRms + 1.0 ) || !qFuzzyCompare( peak + 1.0, m_phaseCurrentPeak + 1.0 ) )
    {
      m_phaseCurrentRms  = rms;
      m_phaseCurrentPeak = peak;
      emit telemetryValuesChanged();
    }
  }

  void AppController::setSystemMode( const QString &mode )
  {
    if( m_systemMode == mode )
    {
      return;
    }
    m_systemMode = mode;
    emit systemModeChanged();
  }

  void AppController::setBusy( bool busyValue )
  {
    Q_UNUSED( busyValue )
    emit busyChanged();
  }

  void AppController::setProgress( const QString &text )
  {
    if( m_progressText == text )
    {
      return;
    }
    m_progressText = text;
    emit progressTextChanged();
  }

  void AppController::notify( Notification severity, const QString &message )
  {
    if( message.isEmpty() )
    {
      return;
    }

    const auto logSeverity = severity == Notification::Error
                                 ? orbit::model::LogModel::Severity::Error
                                 : ( severity == Notification::Warning ? orbit::model::LogModel::Severity::Warning
                                                                        : orbit::model::LogModel::Severity::Info );
    m_logModel->addApp( logSeverity, message );
    emit notification( static_cast<int>( severity ), message );
  }

  void AppController::refreshDirtyCount()
  {
    emit dirtyCountChanged();
  }

  QString AppController::modeName( int mode ) const
  {
    switch( mode )
    {
      case MotorCtrlState_MOTOR_CTRL_STATE_IDLE:
        return "IDLE";
      case MotorCtrlState_MOTOR_CTRL_STATE_ARMED:
        return "ARMED";
      case MotorCtrlState_MOTOR_CTRL_STATE_ENGAGED:
        return "ENGAGED";
      case MotorCtrlState_MOTOR_CTRL_STATE_FAULT:
        return "FAULT";
      case MotorCtrlState_MOTOR_CTRL_STATE_INVALID:
      default:
        return "INVALID";
    }
  }

  orbit::model::ParameterModel *AppController::modelForParamId( int paramId ) const
  {
    const auto *descriptor = orbit::model::registry::find( paramId );
    if( !descriptor )
    {
      return nullptr;
    }

    switch( descriptor->group )
    {
      case orbit::model::ParameterGroup::Information:
        return m_informationModel;
      case orbit::model::ParameterGroup::SystemBehavior:
        return m_systemModel;
      case orbit::model::ParameterGroup::MotorControl:
        return m_motorControlModel;
      case orbit::model::ParameterGroup::MotorAttributes:
        return m_motorAttributesModel;
      default:
        return nullptr;
    }
  }
}    // namespace orbit::app
