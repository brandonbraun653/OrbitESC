/******************************************************************************
 *  File Name:
 *    sim_tcp_server.cpp
 *
 *  Description:
 *    Matlab TCP/IP bridge for simulator builds.
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#if defined( SIMULATOR )

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <src/simulator/sim_tcp_server.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace Orbit::Sim::Matlab
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr size_t SOCKET_BACKLOG = 1;
  static constexpr size_t RX_BUFFER_SIZE = sizeof( RxMessage );
  static constexpr size_t TX_BUFFER_SIZE = sizeof( TxMessage );

  /*---------------------------------------------------------------------------
  Local Types
  ---------------------------------------------------------------------------*/

  struct SocketContext
  {
    int                listen_fd;
    int                client_fd;
    struct sockaddr_in listen_addr;
    socklen_t          addr_len;

    SocketContext()
    {
      listen_fd                   = -1;
      client_fd                   = -1;
      listen_addr                 = {};
      listen_addr.sin_family      = AF_INET;
      listen_addr.sin_addr.s_addr = INADDR_ANY;
      listen_addr.sin_port        = 0;
      addr_len                    = sizeof( listen_addr );
    }

    void closeClient()
    {
      if( client_fd >= 0 )
      {
        ::close( client_fd );
        client_fd = -1;
      }
    }

    void closeAll()
    {
      closeClient();

      if( listen_fd >= 0 )
      {
        ::close( listen_fd );
        listen_fd = -1;
      }
    }
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static std::thread                         s_worker_thread;
  static std::mutex                          s_rx_mutex;
  static std::mutex                          s_tx_mutex;
  static std::mutex                          s_start_stop_mutex;
  static std::atomic<bool>                   s_thread_started{ false };
  static std::atomic<bool>                   s_running{ false };
  static std::atomic<bool>                   s_client_connected{ false };
  static std::atomic<bool>                   s_exit_request{ false };
  static SocketContext                       s_socket_context;
  static RxMessage                           s_rx_message{};
  static TxMessage                           s_tx_message{};
  static std::atomic<bool>                   s_rx_message_ready{ false };
  static std::atomic<bool>                   s_tx_message_dirty{ false };
  static std::array<uint8_t, RX_BUFFER_SIZE> s_rx_staging{};
  static size_t                              s_rx_bytes_pending = 0;
  static std::array<uint8_t, TX_BUFFER_SIZE> s_tx_staging{};
  static size_t                              s_tx_bytes_pending = 0;

  /*---------------------------------------------------------------------------
  Local Functions
  ---------------------------------------------------------------------------*/

  static void tcpServerThread( uint16_t port );
  static bool configureSocket( SocketContext &ctx, const uint16_t port );
  static bool acceptClient( SocketContext &ctx );
  static bool handleRx( SocketContext &ctx );
  static void handleTx( SocketContext &ctx );
  static void cleanup();

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  bool startServer( uint16_t port )
  {
    std::scoped_lock lock( s_start_stop_mutex );

    if( s_thread_started.load() )
    {
      return true;
    }

    s_exit_request.store( false );
    s_thread_started.store( true );

    try
    {
      s_worker_thread = std::thread( tcpServerThread, port );
    }
    catch( const std::exception &e )
    {
      LOG_ERROR( "Matlab TCP server failed to start: %s", e.what() );
      s_thread_started.store( false );
      return false;
    }

    return true;
  }


  void stopServer()
  {
    std::scoped_lock lock( s_start_stop_mutex );

    if( !s_thread_started.load() )
    {
      return;
    }

    s_exit_request.store( true );
    s_socket_context.closeAll();

    if( s_worker_thread.joinable() )
    {
      s_worker_thread.join();
    }

    s_exit_request.store( false );
    s_thread_started.store( false );
  }


  void pushMessage( const TxMessage &message )
  {
    std::scoped_lock lock( s_tx_mutex );
    s_tx_message = message;
    s_tx_message_dirty.store( true );
  }


  bool getLastMessage( RxMessage &message )
  {
    if( !s_rx_message_ready.load() )
    {
      return false;
    }

    std::scoped_lock lock( s_rx_mutex );
    message = s_rx_message;
    s_rx_message_ready.store( false );
    return true;
  }


  bool isRunning()
  {
    return s_running.load();
  }


  bool isClientConnected()
  {
    return s_client_connected.load();
  }

  /*---------------------------------------------------------------------------
  Local Functions
  ---------------------------------------------------------------------------*/

  static void tcpServerThread( const uint16_t port )
  {
    Chimera::Thread::this_thread::set_name( "matlab_tcp" );

    s_running.store( true );
    LOG_INFO( "Matlab TCP server thread starting on port %u", port );

    while( !s_exit_request.load() )
    {
      if( !configureSocket( s_socket_context, port ) )
      {
        LOG_ERROR( "Failed to configure socket" );
        Chimera::delayMilliseconds( 500 );
        continue;
      }

      while( !s_exit_request.load() )
      {
        if( !s_client_connected.load() )
        {
          if( !acceptClient( s_socket_context ) )
          {
            Chimera::delayMilliseconds( 50 );
            continue;
          }

          LOG_INFO( "Matlab client connected" );
          s_client_connected.store( true );
          s_rx_bytes_pending = 0;
          s_tx_bytes_pending = 0;
        }

        if( !handleRx( s_socket_context ) )
        {
          cleanup();
          break;
        }

        handleTx( s_socket_context );
        Chimera::delayMilliseconds( 1 );
      }

      cleanup();
    }

    cleanup();
    s_running.store( false );
    LOG_INFO( "Matlab TCP server thread exiting" );
  }

  static bool configureSocket( SocketContext &ctx, const uint16_t port )
  {
    if( ctx.listen_fd >= 0 )
    {
      return true;
    }

    ctx.listen_fd = ::socket( AF_INET, SOCK_STREAM, 0 );
    if( ctx.listen_fd < 0 )
    {
      LOG_ERROR( "Failed to create socket: %d", errno );
      return false;
    }

    int enable = 1;
    if( ::setsockopt( ctx.listen_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &enable, sizeof( enable ) ) < 0 )
    {
      LOG_ERROR( "Failed to set SO_REUSEADDR: %d", errno );
      return false;
    }

    ctx.listen_addr.sin_port = ::htons( port );
    if( ::bind( ctx.listen_fd, reinterpret_cast<sockaddr *>( &ctx.listen_addr ), sizeof( ctx.listen_addr ) ) < 0 )
    {
      LOG_ERROR( "Failed to bind socket: %d", errno );
      return false;
    }

    if( ::listen( ctx.listen_fd, SOCKET_BACKLOG ) < 0 )
    {
      LOG_ERROR( "Failed to listen on socket: %d", errno );
      return false;
    }

    int flags = ::fcntl( ctx.listen_fd, F_GETFL, 0 );
    if( flags < 0 )
    {
      LOG_ERROR( "Failed to get socket flags: %d", errno );
      return false;
    }

    if( ::fcntl( ctx.listen_fd, F_SETFL, flags | O_NONBLOCK ) < 0 )
    {
      LOG_ERROR( "Failed to set listen socket non-blocking: %d", errno );
      return false;
    }

    return true;
  }

  static bool acceptClient( SocketContext &ctx )
  {
    ctx.client_fd = ::accept( ctx.listen_fd, reinterpret_cast<sockaddr *>( &ctx.listen_addr ), &ctx.addr_len );

    if( ctx.client_fd < 0 )
    {
      if( ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
      {
        LOG_ERROR( "Failed to accept client: %d", errno );
      }

      return false;
    }

    int flags = ::fcntl( ctx.client_fd, F_GETFL, 0 );
    if( flags < 0 )
    {
      LOG_ERROR( "Failed to get client socket flags: %d", errno );
      return false;
    }

    if( ::fcntl( ctx.client_fd, F_SETFL, flags | O_NONBLOCK ) < 0 )
    {
      LOG_ERROR( "Failed to set client socket non-blocking: %d", errno );
      return false;
    }

    return true;
  }

  static bool handleRx( SocketContext &ctx )
  {
    if( ctx.client_fd < 0 )
    {
      return false;
    }

    bool connection_ok = true;

    while( true )
    {
      const size_t bytes_needed = RX_BUFFER_SIZE - s_rx_bytes_pending;

      if( !bytes_needed )
      {
        break;
      }

      ssize_t bytes_read = ::recv( ctx.client_fd, s_rx_staging.data() + s_rx_bytes_pending, bytes_needed, MSG_DONTWAIT );

      if( bytes_read == 0 )
      {
        LOG_WARN( "Matlab client disconnected" );
        connection_ok = false;
        break;
      }
      else if( bytes_read < 0 )
      {
        if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
        {
          break;
        }

        LOG_ERROR( "Socket recv failed: %d", errno );
        connection_ok = false;
        break;
      }

      s_rx_bytes_pending += static_cast<size_t>( bytes_read );

      if( s_rx_bytes_pending == RX_BUFFER_SIZE )
      {
        std::scoped_lock lock( s_rx_mutex );
        std::memcpy( &s_rx_message, s_rx_staging.data(), RX_BUFFER_SIZE );
        s_rx_message_ready.store( true );
        s_rx_bytes_pending = 0;
      }
    }

    return connection_ok;
  }

  static void handleTx( SocketContext &ctx )
  {
    if( ctx.client_fd < 0 )
    {
      return;
    }

    if( s_tx_message_dirty.load() && ( s_tx_bytes_pending == 0 ) )
    {
      std::scoped_lock lock( s_tx_mutex );
      std::memcpy( s_tx_staging.data(), &s_tx_message, TX_BUFFER_SIZE );
      s_tx_message_dirty.store( false );
      s_tx_bytes_pending = TX_BUFFER_SIZE;
    }

    if( s_tx_bytes_pending == 0 )
    {
      return;
    }

    const size_t  offset = TX_BUFFER_SIZE - s_tx_bytes_pending;
    const ssize_t bytes_sent =
        ::send( ctx.client_fd, s_tx_staging.data() + offset, s_tx_bytes_pending, MSG_NOSIGNAL | MSG_DONTWAIT );

    if( bytes_sent < 0 )
    {
      if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
      {
        return;
      }

      if( ( errno == EPIPE ) || ( errno == ECONNRESET ) )
      {
        LOG_WARN( "Matlab client disconnected during send" );
      }
      else
      {
        LOG_ERROR( "Socket send failed: %d", errno );
      }

      cleanup();
      return;
    }

    s_tx_bytes_pending -= static_cast<size_t>( bytes_sent );
  }

  static void cleanup()
  {
    s_socket_context.closeAll();
    s_client_connected.store( false );
  }

}    // namespace Orbit::Sim::Matlab

#endif /* SIMULATOR */
