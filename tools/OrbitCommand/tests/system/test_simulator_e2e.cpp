#include <gtest/gtest.h>

#include <QElapsedTimer>
#include <QHostAddress>
#include <QMetaObject>

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include "device/DeviceClient.h"
#include "device/TcpTransport.h"
#include "protocol/ParameterCodec.h"
#include "support/SimulatorProcess.h"
#include "support/Wait.h"

using namespace orbit::device;
using namespace orbit::protocol;
using orbit::test::processEventsFor;
using orbit::test::SimulatorProcess;
using orbit::test::waitFor;

namespace
{
  int envInt( const char *name, int fallback )
  {
    bool ok        = false;
    const int value = qEnvironmentVariableIntValue( name, &ok );
    return ok ? value : fallback;
  }

  ConnectionTarget simulatorTarget( quint16 port )
  {
    ConnectionTarget target;
    target.kind        = QStringLiteral( "simulator" );
    target.id          = QStringLiteral( "simulator" );
    target.displayName = QStringLiteral( "Simulator" );
    target.properties.insert( "host", QStringLiteral( "127.0.0.1" ) );
    target.properties.insert( "port", port );
    return target;
  }

  void logMetric( const char *name, double value )
  {
    std::cout << "[ METRIC   ] " << name << " = " << value << std::endl;
    ::testing::Test::RecordProperty( name, value );
  }

  double rateOverWindow( const QList<qint64> &stamps, int windowMs )
  {
    if( windowMs <= 0 )
    {
      return 0.0;
    }
    return static_cast<double>( stamps.size() ) * 1000.0 / static_cast<double>( windowMs );
  }

  qint64 maximumGap( const QList<qint64> &stamps )
  {
    qint64 gap = 0;
    for( qsizetype index = 1; index < stamps.size(); ++index )
    {
      gap = std::max( gap, stamps.at( index ) - stamps.at( index - 1 ) );
    }
    return gap;
  }
}    // namespace

class SimulatorE2E : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if( SimulatorProcess::resolveBinary().isEmpty() )
    {
      GTEST_SKIP() << "OrbitESC simulator binary not found; set ORBIT_SIMULATOR_BINARY";
    }

    s_port = SimulatorProcess::findFreePort();
    if( s_port == 0 || !s_simulator.start( s_port ) )
    {
      GTEST_SKIP() << "Could not start the OrbitESC simulator on a free TCP port";
    }
    s_available = true;
  }

  static void TearDownTestSuite()
  {
    if( s_available )
    {
      s_simulator.stop();
      s_available = false;
    }
  }

  bool connectClient( DeviceClient &client, TcpTransport &transport, int timeoutMs )
  {
    return transport.open( simulatorTarget( s_port ) ) &&
           waitFor( [&client]() { return client.state() == DeviceClient::State::Online; }, timeoutMs );
  }

  static SimulatorProcess s_simulator;
  static quint16 s_port;
  static bool s_available;
};

SimulatorProcess SimulatorE2E::s_simulator;
quint16 SimulatorE2E::s_port = 0;
bool SimulatorE2E::s_available  = false;

TEST_F( SimulatorE2E, ConnectAndMeasureRates )
{
  const int maxConnectMs = envInt( "ORBIT_E2E_MAX_CONNECT_MS", 3000 );
  const int windowMs     = 5000;
  const int warmupMs     = 1000;

  TcpTransport transport;
  DeviceClient client( &transport );

  QElapsedTimer connectTimer;
  connectTimer.start();
  ASSERT_TRUE( transport.open( simulatorTarget( s_port ) ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; }, maxConnectMs ) );
  const qint64 connectMs = connectTimer.elapsed();
  logMetric( "connect_ms", static_cast<double>( connectMs ) );
  EXPECT_LE( connectMs, maxConnectMs );

  QElapsedTimer window;
  QList<qint64> tickStamps;
  QList<qint64> statusStamps;
  QList<qint64> voltageStamps;

  const auto tickConnection = QObject::connect( &client, &DeviceClient::linkTick,
                                                [&window, &tickStamps]( quint32 )
                                                {
                                                  tickStamps.append( window.elapsed() );
                                                } );
  const auto statusConnection = QObject::connect( &client, &DeviceClient::systemStatusChanged,
                                                  [&window, &statusStamps]( int, quint32 )
                                                  {
                                                    statusStamps.append( window.elapsed() );
                                                  } );
  const auto voltageConnection = QObject::connect( &client, &DeviceClient::systemVoltagesReceived,
                                                   [&window, &voltageStamps]( float, float, float, float )
                                                   {
                                                     voltageStamps.append( window.elapsed() );
                                                   } );

  window.start();
  processEventsFor( warmupMs );
  tickStamps.clear();
  statusStamps.clear();
  voltageStamps.clear();
  window.restart();
  processEventsFor( windowMs );

  QObject::disconnect( tickConnection );
  QObject::disconnect( statusConnection );
  QObject::disconnect( voltageConnection );

  const double tickRate    = rateOverWindow( tickStamps, windowMs );
  const double statusRate  = rateOverWindow( statusStamps, windowMs );
  const double voltageRate = rateOverWindow( voltageStamps, windowMs );
  const qint64 maxTickGap  = maximumGap( tickStamps );

  logMetric( "tick_hz", tickRate );
  logMetric( "status_hz", statusRate );
  logMetric( "voltage_hz", voltageRate );
  logMetric( "max_tick_gap_ms", static_cast<double>( maxTickGap ) );

  EXPECT_GE( tickRate, envInt( "ORBIT_E2E_MIN_TICK_HZ", 8 ) );
  EXPECT_LE( tickRate, envInt( "ORBIT_E2E_MAX_TICK_HZ", 12 ) );
  EXPECT_LE( maxTickGap, envInt( "ORBIT_E2E_MAX_TICK_GAP_MS", 1500 ) );
  EXPECT_GE( statusRate, envInt( "ORBIT_E2E_MIN_STATUS_HZ", 4 ) );
  EXPECT_LE( statusRate, envInt( "ORBIT_E2E_MAX_STATUS_HZ", 6 ) );
  EXPECT_GE( voltageRate, envInt( "ORBIT_E2E_MIN_VOLTAGE_HZ", 4 ) );
  EXPECT_LE( voltageRate, envInt( "ORBIT_E2E_MAX_VOLTAGE_HZ", 6 ) );
}

TEST_F( SimulatorE2E, ParameterRoundTrip )
{
  const int maxConnectMs = envInt( "ORBIT_E2E_MAX_CONNECT_MS", 3000 );
  const int maxRttMs     = envInt( "ORBIT_E2E_MAX_PARAM_RTT_MS", 2000 );

  TcpTransport transport;
  DeviceClient client( &transport );
  ASSERT_TRUE( connectClient( client, transport, maxConnectMs ) );

  QElapsedTimer timer;
  bool received = false;
  const auto connection = QObject::connect( &client, &DeviceClient::parameterValueReceived,
                                            [&received]( int paramId, int, const QByteArray & )
                                            {
                                              if( paramId == ParamId_PARAM_BOOT_COUNT )
                                              {
                                                received = true;
                                              }
                                            } );

  timer.start();
  client.requestParameterGet( ParamId_PARAM_BOOT_COUNT );
  ASSERT_TRUE( waitFor( [&received]() { return received; }, maxRttMs ) );
  const qint64 rttMs = timer.elapsed();
  QObject::disconnect( connection );

  logMetric( "parameter_rtt_ms", static_cast<double>( rttMs ) );
  EXPECT_LE( rttMs, maxRttMs );
}

TEST_F( SimulatorE2E, ControlTransitions )
{
  const int maxConnectMs = envInt( "ORBIT_E2E_MAX_CONNECT_MS", 3000 );
  const int maxControlMs = envInt( "ORBIT_E2E_MAX_CONTROL_MS", 1000 );

  TcpTransport transport;
  DeviceClient client( &transport );
  ASSERT_TRUE( connectClient( client, transport, maxConnectMs ) );

  QList<int> observedStates;
  const auto connection = QObject::connect( &client, &DeviceClient::systemStatusChanged,
                                            [&observedStates]( int state, quint32 )
                                            {
                                              observedStates.append( state );
                                            } );

  auto waitForState = [&observedStates]( int state, int timeoutMs )
  {
    return waitFor(
        [&observedStates, state]()
        {
          return std::find( observedStates.begin(), observedStates.end(), state ) != observedStates.end();
        },
        timeoutMs );
  };

  QElapsedTimer timer;
  bool armAcknowledged = false;
  const auto armConnection = QObject::connect( &client, &DeviceClient::commandFinished,
                                               [&armAcknowledged]( DeviceClient::Command command, int detail, bool ok,
                                                                   const QString & )
                                               {
                                                 if( command == DeviceClient::Command::SystemControl &&
                                                     detail == SystemControlSubId_ARM )
                                                 {
                                                   armAcknowledged = ok;
                                                 }
                                               } );

  timer.start();
  client.requestSystemControl( SystemControlSubId_ARM, SystemControlSubId_ARM );
  ASSERT_TRUE( waitFor( [&armAcknowledged]() { return armAcknowledged; }, maxControlMs ) );
  const qint64 armMs = timer.elapsed();
  logMetric( "arm_ms", static_cast<double>( armMs ) );
  EXPECT_LE( armMs, maxControlMs );
  EXPECT_TRUE( waitForState( MotorCtrlState_MOTOR_CTRL_STATE_ARMED, maxControlMs ) );
  QObject::disconnect( armConnection );

  bool engageAcknowledged = false;
  const auto engageConnection = QObject::connect( &client, &DeviceClient::commandFinished,
                                                  [&engageAcknowledged]( DeviceClient::Command command, int detail,
                                                                         bool ok, const QString & )
                                                  {
                                                    if( command == DeviceClient::Command::SystemControl &&
                                                        detail == SystemControlSubId_ENGAGE )
                                                    {
                                                      engageAcknowledged = ok;
                                                    }
                                                  } );
  client.requestSystemControl( SystemControlSubId_ENGAGE, SystemControlSubId_ENGAGE );
  ASSERT_TRUE( waitFor( [&engageAcknowledged]() { return engageAcknowledged; }, maxControlMs ) );
  QObject::disconnect( engageConnection );

  bool faultAcknowledged = false;
  const auto faultConnection = QObject::connect( &client, &DeviceClient::commandFinished,
                                                 [&faultAcknowledged]( DeviceClient::Command command, int detail, bool ok,
                                                                       const QString & )
                                                 {
                                                   if( command == DeviceClient::Command::SystemControl &&
                                                       detail == SystemControlSubId_FAULT )
                                                   {
                                                     faultAcknowledged = ok;
                                                   }
                                                 } );
  timer.restart();
  client.requestSystemControl( SystemControlSubId_FAULT, SystemControlSubId_FAULT );
  ASSERT_TRUE( waitFor( [&faultAcknowledged]() { return faultAcknowledged; }, maxControlMs ) );
  const qint64 faultMs = timer.elapsed();
  logMetric( "fault_ms", static_cast<double>( faultMs ) );
  EXPECT_LE( faultMs, maxControlMs );
  EXPECT_TRUE( waitForState( MotorCtrlState_MOTOR_CTRL_STATE_FAULT, maxControlMs ) );
  QObject::disconnect( faultConnection );

  bool disableAcknowledged = false;
  const auto disableConnection = QObject::connect( &client, &DeviceClient::commandFinished,
                                                   [&disableAcknowledged]( DeviceClient::Command command, int detail,
                                                                           bool ok, const QString & )
                                                   {
                                                     if( command == DeviceClient::Command::SystemControl &&
                                                         detail == SystemControlSubId_DISABLE )
                                                     {
                                                       disableAcknowledged = ok;
                                                     }
                                                   } );
  client.requestSystemControl( SystemControlSubId_DISABLE, SystemControlSubId_DISABLE );
  ASSERT_TRUE( waitFor( [&disableAcknowledged]() { return disableAcknowledged; }, maxControlMs ) );
  QObject::disconnect( disableConnection );
  QObject::disconnect( connection );
}

TEST_F( SimulatorE2E, StreamRate )
{
  const int maxConnectMs = envInt( "ORBIT_E2E_MAX_CONNECT_MS", 3000 );
  const int windowMs     = 5000;
  const int warmupMs     = 1000;

  TcpTransport transport;
  DeviceClient client( &transport );
  ASSERT_TRUE( connectClient( client, transport, maxConnectMs ) );

  bool streamAcknowledged = false;
  const auto commandConnection = QObject::connect( &client, &DeviceClient::commandFinished,
                                                   [&streamAcknowledged]( DeviceClient::Command command, int, bool ok,
                                                                          const QString & )
                                                   {
                                                     if( command == DeviceClient::Command::Stream )
                                                     {
                                                       streamAcknowledged = ok;
                                                     }
                                                   } );
  client.setPhaseCurrentStream( true );
  ASSERT_TRUE( waitFor( [&streamAcknowledged]() { return streamAcknowledged; },
                        envInt( "ORBIT_E2E_MAX_CONTROL_MS", 1000 ) ) );
  QObject::disconnect( commandConnection );

  QElapsedTimer window;
  QList<qint64> stamps;
  const auto sampleConnection = QObject::connect( &client, &DeviceClient::phaseCurrentsReceived,
                                                  [&window, &stamps]( float, float, float, quint32 )
                                                  {
                                                    stamps.append( window.elapsed() );
                                                  } );

  window.start();
  processEventsFor( warmupMs );
  stamps.clear();
  window.restart();
  processEventsFor( windowMs );
  QObject::disconnect( sampleConnection );

  const double currentRate = rateOverWindow( stamps, windowMs );
  logMetric( "current_hz", currentRate );
  EXPECT_GE( currentRate, envInt( "ORBIT_E2E_MIN_CURRENT_HZ", 25 ) );
  EXPECT_LE( currentRate, envInt( "ORBIT_E2E_MAX_CURRENT_HZ", 35 ) );

  client.setPhaseCurrentStream( false );
  processEventsFor( 500 );
}

TEST_F( SimulatorE2E, ReconnectAfterSimulatorRestart )
{
  const int maxConnectMs = envInt( "ORBIT_E2E_MAX_CONNECT_MS", 3000 );

  TcpTransport transport;
  DeviceClient client( &transport );
  ASSERT_TRUE( connectClient( client, transport, maxConnectMs ) );

  s_simulator.stop();
  ASSERT_TRUE( waitFor( [&client]() { return client.state() != DeviceClient::State::Online; }, 5000 ) );

  ASSERT_TRUE( s_simulator.restart( 20000 ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; }, 20000 ) );
}
