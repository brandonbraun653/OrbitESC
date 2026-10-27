#include <gtest/gtest.h>

#include <QTcpServer>
#include <QTcpSocket>

#include "device/TcpTransport.h"
#include "support/SignalRecorder.h"
#include "support/Wait.h"

using namespace orbit::device;
using orbit::test::processEventsFor;
using orbit::test::recordSignal;
using orbit::test::waitFor;

namespace
{
  ConnectionTarget targetFor( quint16 port )
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

TEST( TcpTransport, ConnectsReadsWritesAndCloses )
{
  QTcpServer server;
  ASSERT_TRUE( server.listen( QHostAddress::LocalHost, 0 ) );

  TcpTransport transport;
  auto opened = recordSignal<>( &transport, &Transport::opened );
  auto closed = recordSignal<>( &transport, &Transport::closed );
  auto data   = recordSignal<QByteArray>( &transport, &Transport::dataReceived );

  ASSERT_TRUE( transport.open( targetFor( server.serverPort() ) ) );
  ASSERT_TRUE( waitFor( [&opened]() { return opened.count() == 1; } ) );
  EXPECT_TRUE( transport.isOpen() );

  ASSERT_TRUE( waitFor( [&server]() { return server.hasPendingConnections(); } ) );
  QTcpSocket *peer = server.nextPendingConnection();
  ASSERT_NE( peer, nullptr );

  transport.write( QByteArrayLiteral( "ping" ) );
  ASSERT_TRUE( waitFor( [peer]() { return peer->bytesAvailable() >= 4; } ) );
  EXPECT_EQ( peer->readAll(), QByteArrayLiteral( "ping" ) );

  peer->write( QByteArrayLiteral( "pong" ) );
  ASSERT_TRUE( waitFor( [&data]() { return data.count() == 1; } ) );
  EXPECT_EQ( std::get<0>( data.first() ), QByteArrayLiteral( "pong" ) );

  transport.close();
  EXPECT_EQ( closed.count(), 1 );
  EXPECT_FALSE( transport.isOpen() );
}

TEST( TcpTransport, TransientDisconnectDoesNotEmitClosed )
{
  QTcpServer server;
  ASSERT_TRUE( server.listen( QHostAddress::LocalHost, 0 ) );

  TcpTransport transport;
  auto closed = recordSignal<>( &transport, &Transport::closed );

  ASSERT_TRUE( transport.open( targetFor( server.serverPort() ) ) );
  ASSERT_TRUE( waitFor( [&server]() { return server.hasPendingConnections(); } ) );
  QTcpSocket *peer = server.nextPendingConnection();
  ASSERT_NE( peer, nullptr );

  peer->disconnectFromHost();
  processEventsFor( 100 );
  EXPECT_EQ( closed.count(), 0 );

  transport.close();
  EXPECT_EQ( closed.count(), 1 );
}

TEST( TcpTransport, RefusedConnectionRetriesUntilClosed )
{
  QTcpServer reserved;
  ASSERT_TRUE( reserved.listen( QHostAddress::LocalHost, 0 ) );
  const quint16 closedPort = reserved.serverPort();
  reserved.close();

  TcpTransport transport;
  auto errors = recordSignal<QString>( &transport, &Transport::errorOccurred );
  auto closed = recordSignal<>( &transport, &Transport::closed );

  ASSERT_TRUE( transport.open( targetFor( closedPort ) ) );
  ASSERT_TRUE( waitFor( [&errors]() { return errors.count() >= 1; } ) );
  EXPECT_EQ( closed.count(), 0 );

  transport.close();
  EXPECT_EQ( closed.count(), 1 );

  processEventsFor( 600 );
  EXPECT_EQ( closed.count(), 1 );
}
