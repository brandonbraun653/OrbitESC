#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <QTcpServer>
#include <QTcpSocket>

#include "device/DeviceClient.h"
#include "device/TcpTransport.h"
#include "protocol/FrameCodec.h"
#include "protocol/MessageCodec.h"
#include "protocol/ParameterCodec.h"
#include "support/FakeDeviceServer.h"
#include "support/FakeTransport.h"
#include "support/MessageFactory.h"
#include "support/MockTransport.h"
#include "support/SignalRecorder.h"
#include "support/Wait.h"

using namespace orbit::device;
using namespace orbit::protocol;
using orbit::test::FakeDeviceServer;
using orbit::test::FakeTransport;
using orbit::test::recordSignal;
using orbit::test::waitFor;

namespace
{
  ConnectionTarget targetFor( const QString &portName )
  {
    ConnectionTarget target;
    target.kind                   = QStringLiteral( "serial" );
    target.id                     = QStringLiteral( "serial:%1" ).arg( portName );
    target.displayName            = portName;
    target.properties["portName"] = portName;
    return target;
  }

  ConnectionTarget simulatorTargetFor( quint16 port )
  {
    ConnectionTarget target;
    target.kind        = QStringLiteral( "simulator" );
    target.id          = QStringLiteral( "simulator" );
    target.displayName = QStringLiteral( "Simulator" );
    target.properties.insert( "host", QStringLiteral( "127.0.0.1" ) );
    target.properties.insert( "port", port );
    return target;
  }
}    // namespace

TEST( DeviceClient, ConnectPingAndTick )
{
  FakeTransport transport;
  DeviceClient client( &transport );
  auto states = recordSignal<DeviceClient::State>( &client, &DeviceClient::stateChanged );

  transport.onWrite = [&transport]( const QByteArray & )
  {
    const auto header = transport.lastHeader();
    if( header.has_value() && header->msgId == MsgId_MSG_PING_CMD )
    {
      transport.inject( orbit::test::ackFor( *header ) );
    }
  };

  client.openTarget( targetFor( "fake" ) );
  ASSERT_TRUE( waitFor( [&states]() { return states.count() >= 1; } ) );
  EXPECT_EQ( client.state(), DeviceClient::State::Connecting );
  ASSERT_TRUE( waitFor( [&transport]() { return transport.writes.size() >= 1; } ) );

  transport.inject( orbit::test::tickMessage( 1 ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );
}

TEST( DeviceClient, ConnectsOverTcp )
{
  FakeDeviceServer server;
  ASSERT_TRUE( server.start( 0 ) );

  TcpTransport transport;
  DeviceClient client( &transport );
  ASSERT_TRUE( transport.open( simulatorTargetFor( server.port() ) ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );
}

TEST( DeviceClient, ParameterResponseIsCorrelated )
{
  FakeTransport transport;
  DeviceClient client( &transport );

  transport.onWrite = [&transport]( const QByteArray & )
  {
    const auto header = transport.lastHeader();
    if( !header.has_value() )
    {
      return;
    }

    if( header->msgId == MsgId_MSG_PING_CMD )
    {
      transport.inject( orbit::test::ackFor( *header ) );
    }
    else if( header->msgId == MsgId_MSG_PARAM_IO && header->subId == ParamIOSubId_GET )
    {
      bool encoded          = false;
      const QByteArray data = encodeParameterValue( ParamType_UINT32, 9, &encoded );
      if( encoded )
      {
        transport.inject(
            orbit::test::parameterReply( header->uuid, ParamId_PARAM_BOOT_COUNT, ParamType_UINT32, data ) );
      }
    }
  };

  client.openTarget( targetFor( "fake" ) );
  ASSERT_TRUE( waitFor( [&transport]() { return transport.writes.size() >= 1; } ) );
  transport.inject( orbit::test::tickMessage( 1 ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );

  auto values = recordSignal<int, int, QByteArray>( &client, &DeviceClient::parameterValueReceived );
  auto done   = recordSignal<DeviceClient::Command, int, bool, QString>( &client, &DeviceClient::commandFinished );
  client.requestParameterGet( ParamId_PARAM_BOOT_COUNT );

  ASSERT_TRUE( waitFor( [&values]() { return values.count() >= 1; } ) );
  EXPECT_EQ( std::get<0>( values.first() ), static_cast<int>( ParamId_PARAM_BOOT_COUNT ) );
  EXPECT_GE( done.count(), 1 );
}

TEST( DeviceClient, StreamCommandUsesCorrectSubId )
{
  FakeTransport transport;
  DeviceClient client( &transport );

  transport.onWrite = [&transport]( const QByteArray & )
  {
    const auto header = transport.lastHeader();
    if( header.has_value() )
    {
      transport.inject( orbit::test::ackFor( *header ) );
    }
  };

  client.openTarget( targetFor( "fake" ) );
  ASSERT_TRUE( waitFor( [&transport]() { return transport.writes.size() >= 1; } ) );
  transport.inject( orbit::test::tickMessage( 1 ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );

  auto done = recordSignal<DeviceClient::Command, int, bool, QString>( &client, &DeviceClient::commandFinished );
  client.setPhaseCurrentStream( true );

  ASSERT_TRUE( waitFor( [&done]() { return done.count() >= 1; } ) );
  const auto header = transport.lastHeader();
  ASSERT_TRUE( header.has_value() );
  EXPECT_EQ( header->msgId, static_cast<quint8>( MsgId_MSG_SYS_CTRL ) );
  EXPECT_EQ( header->subId, static_cast<quint8>( SystemControlSubId_ENABLE_STREAM_PHASE_CURRENTS ) );
}

TEST( DeviceClient, ParameterWriteIsAcknowledged )
{
  FakeTransport transport;
  DeviceClient client( &transport );

  transport.onWrite = [&transport]( const QByteArray & )
  {
    const auto header = transport.lastHeader();
    if( header.has_value() )
    {
      transport.inject( orbit::test::ackFor( *header ) );
    }
  };

  client.openTarget( targetFor( "fake" ) );
  ASSERT_TRUE( waitFor( [&transport]() { return transport.writes.size() >= 1; } ) );
  transport.inject( orbit::test::tickMessage( 1 ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );

  bool encoded          = false;
  const QByteArray data = encodeParameterValue( ParamType_FLOAT, 1.25f, &encoded );
  ASSERT_TRUE( encoded );

  auto writes = recordSignal<int, bool, QString>( &client, &DeviceClient::parameterWriteFinished );
  client.requestParameterSet( ParamId_PARAM_ACTIVITY_LED_SCALER, ParamType_FLOAT, data );

  ASSERT_TRUE( waitFor( [&writes]() { return writes.count() >= 1; } ) );
  EXPECT_EQ( std::get<0>( writes.first() ), static_cast<int>( ParamId_PARAM_ACTIVITY_LED_SCALER ) );
  EXPECT_TRUE( std::get<1>( writes.first() ) );

  const auto header = transport.lastHeader();
  ASSERT_TRUE( header.has_value() );
  EXPECT_EQ( header->msgId, static_cast<quint8>( MsgId_MSG_PARAM_IO ) );
  EXPECT_EQ( header->subId, static_cast<quint8>( ParamIOSubId_SET ) );
}

TEST( DeviceClient, TickTimeoutGoesOffline )
{
  FakeTransport transport;
  DeviceClient client( &transport );

  transport.onWrite = [&transport]( const QByteArray & )
  {
    const auto header = transport.lastHeader();
    if( header.has_value() && header->msgId == MsgId_MSG_PING_CMD )
    {
      transport.inject( orbit::test::ackFor( *header ) );
    }
  };

  client.openTarget( targetFor( "fake" ) );
  ASSERT_TRUE( waitFor( [&transport]() { return transport.writes.size() >= 1; } ) );
  transport.inject( orbit::test::tickMessage( 1 ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );

  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Offline; }, 5000 ) );
}

TEST( DeviceClient, CommandsAreRejectedWhenOffline )
{
  FakeTransport transport;
  DeviceClient client( &transport );
  auto done = recordSignal<DeviceClient::Command, int, bool, QString>( &client, &DeviceClient::commandFinished );

  client.requestParameterGet( ParamId_PARAM_BOOT_COUNT );
  ASSERT_EQ( done.count(), 1 );
  EXPECT_FALSE( std::get<2>( done.first() ) );
}

TEST( DeviceClient, UsesTransportThroughGoogleMock )
{
  ::testing::NiceMock<orbit::test::MockTransport> transport;
  DeviceClient client( &transport );

  QByteArray written;
  EXPECT_CALL( transport, open( ::testing::_ ) ).WillOnce( ::testing::Return( true ) );
  EXPECT_CALL( transport, write( ::testing::_ ) )
      .WillOnce( ::testing::Invoke( [&written]( const QByteArray &data ) { written = data; } ) );
  EXPECT_CALL( transport, close() ).Times( 1 );

  client.openTarget( targetFor( "mock" ) );
  EXPECT_EQ( client.state(), DeviceClient::State::Connecting );

  transport.emitOpened();
  ASSERT_TRUE( waitFor( [&written]() { return !written.isEmpty(); } ) );

  FrameAccumulator accumulator;
  accumulator.append( written );
  const auto frames = accumulator.takeFrames();
  ASSERT_EQ( frames.payloads.size(), 1 );
  const auto header = peekHeader( frames.payloads.first() );
  ASSERT_TRUE( header.has_value() );

  transport.emitData( encodeFrame( orbit::test::ackFor( *header ) ) );
  transport.emitData( encodeFrame( orbit::test::tickMessage( 1 ) ) );
  ASSERT_TRUE( waitFor( [&client]() { return client.state() == DeviceClient::State::Online; } ) );

  client.closeConnection();
  EXPECT_EQ( client.state(), DeviceClient::State::Disconnected );
}
