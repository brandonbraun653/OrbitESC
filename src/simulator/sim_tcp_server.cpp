/******************************************************************************
 *  File Name:
 *    sim_tcp_server.cpp
 *
 *  Description:
 *    Object-oriented TCP/IP server for simulator builds. Supports multiple
 *    ports and arbitrary binary data transmission.
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

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace Orbit::Sim::TCP
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr size_t MAX_TX_QUEUE_SIZE      = 10;
  static constexpr size_t DEFAULT_RX_BUFFER_SIZE = 1024;
  static constexpr size_t DEFAULT_TX_BUFFER_SIZE = 1024;

  /*---------------------------------------------------------------------------
  Server Implementation
  ---------------------------------------------------------------------------*/

  struct Server::SocketContext
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

  struct Server::Impl
  {
    ServerConfig                     config;
    std::thread                      worker_thread;
    std::mutex                       callback_mutex;
    std::mutex                       tx_mutex;
    std::mutex                       start_stop_mutex;
    std::atomic<bool>                thread_started{ false };
    std::atomic<bool>                running{ false };
    std::atomic<bool>                client_connected{ false };
    std::atomic<bool>                exit_request{ false };
    SocketContext                    socket_context;
    std::vector<uint8_t>             rx_buffer;
    std::vector<uint8_t>             tx_buffer;
    std::queue<std::vector<uint8_t>> tx_queue;
    size_t                           rx_bytes_pending{ 0 };
    size_t                           tx_bytes_pending{ 0 };
    DataReceivedCallback             rx_callback;
    ConnectionCallback               connection_callback;
    Server                          *server_ref;

    Impl( const ServerConfig &cfg, Server *server ) :
        config( cfg ), rx_buffer( cfg.rx_buffer_size ), tx_buffer( cfg.tx_buffer_size ), rx_callback( cfg.rx_callback ),
        connection_callback( cfg.connection_callback ), server_ref( server )
    {
    }

    ~Impl()
    {
      stop();
    }

    bool start()
    {
      std::scoped_lock lock( start_stop_mutex );

      if( thread_started.load() )
      {
        return true;
      }

      exit_request.store( false );
      thread_started.store( true );

      try
      {
        worker_thread = std::thread( &Impl::tcpServerThread, this );
      }
      catch( const std::exception &e )
      {
        LOG_ERROR( "TCP server failed to start on port %u: %s", config.port, e.what() );
        thread_started.store( false );
        return false;
      }

      return true;
    }

    void stop()
    {
      std::scoped_lock lock( start_stop_mutex );

      if( !thread_started.load() )
      {
        return;
      }

      exit_request.store( true );
      socket_context.closeAll();

      if( worker_thread.joinable() )
      {
        worker_thread.join();
      }

      exit_request.store( false );
      thread_started.store( false );
    }

    bool sendData( const void *data, size_t size )
    {
      if( !running.load() || !data || !size )
      {
        return false;
      }

      std::scoped_lock lock( tx_mutex );

      if( tx_queue.size() >= MAX_TX_QUEUE_SIZE )
      {
        return false;    // Queue full
      }

      std::vector<uint8_t> packet( static_cast<const uint8_t *>( data ), static_cast<const uint8_t *>( data ) + size );
      tx_queue.push( std::move( packet ) );

      return true;
    }

    bool isRunning() const
    {
      return running.load();
    }

    bool isClientConnected() const
    {
      return client_connected.load();
    }

    uint16_t getPort() const
    {
      return config.port;
    }

    void setRxCallback( DataReceivedCallback callback )
    {
      std::scoped_lock lock( callback_mutex );
      rx_callback = callback;
    }

    void setConnectionCallback( ConnectionCallback callback )
    {
      std::scoped_lock lock( callback_mutex );
      connection_callback = callback;
    }

  private:
    void tcpServerThread()
    {
      Chimera::Thread::this_thread::set_name( ( "tcp_server_" + std::to_string( config.port ) ).c_str() );

      running.store( true );
      LOG_TRACE( "TCP server thread starting on port %u", config.port );

      while( !exit_request.load() )
      {
        if( !configureSocket() )
        {
          LOG_ERROR( "Failed to configure socket on port %u", config.port );
          std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
          continue;
        }

        while( !exit_request.load() )
        {
          if( !client_connected.load() )
          {
            if( !acceptClient() )
            {
              std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
              continue;
            }

            LOG_TRACE( "TCP client connected on port %u", config.port );
            client_connected.store( true );
            rx_bytes_pending = 0;
            tx_bytes_pending = 0;

            if( connection_callback )
            {
              std::scoped_lock lock( callback_mutex );
              connection_callback( *server_ref, ConnectionState::Connected );
            }
          }

          if( !handleRx() )
          {
            cleanup();
            break;
          }

          handleTx();
          std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        }

        cleanup();
      }

      cleanup();
      running.store( false );
      LOG_TRACE( "TCP server thread exiting on port %u", config.port );
    }

    bool configureSocket()
    {
      if( socket_context.listen_fd >= 0 )
      {
        return true;
      }

      socket_context.listen_fd = ::socket( AF_INET, SOCK_STREAM, 0 );
      if( socket_context.listen_fd < 0 )
      {
        LOG_ERROR( "Failed to create socket on port %u: %d", config.port, errno );
        return false;
      }

      int enable = 1;
      if( ::setsockopt( socket_context.listen_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &enable, sizeof( enable ) ) < 0 )
      {
        LOG_ERROR( "Failed to set SO_REUSEADDR on port %u: %d", config.port, errno );
        return false;
      }

      socket_context.listen_addr.sin_port = ::htons( config.port );
      if( ::bind( socket_context.listen_fd, reinterpret_cast<sockaddr *>( &socket_context.listen_addr ),
                  sizeof( socket_context.listen_addr ) ) < 0 )
      {
        LOG_ERROR( "Failed to bind socket on port %u: %d", config.port, errno );
        return false;
      }

      if( ::listen( socket_context.listen_fd, static_cast<int>( config.socket_backlog ) ) < 0 )
      {
        LOG_ERROR( "Failed to listen on socket port %u: %d", config.port, errno );
        return false;
      }

      int flags = ::fcntl( socket_context.listen_fd, F_GETFL, 0 );
      if( flags < 0 )
      {
        LOG_ERROR( "Failed to get socket flags on port %u: %d", config.port, errno );
        return false;
      }

      if( ::fcntl( socket_context.listen_fd, F_SETFL, flags | O_NONBLOCK ) < 0 )
      {
        LOG_ERROR( "Failed to set listen socket non-blocking on port %u: %d", config.port, errno );
        return false;
      }

      return true;
    }

    bool acceptClient()
    {
      socket_context.client_fd = ::accept(
          socket_context.listen_fd, reinterpret_cast<sockaddr *>( &socket_context.listen_addr ), &socket_context.addr_len );

      if( socket_context.client_fd < 0 )
      {
        if( ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
        {
          LOG_ERROR( "Failed to accept client on port %u: %d", config.port, errno );
        }

        return false;
      }

      int flags = ::fcntl( socket_context.client_fd, F_GETFL, 0 );
      if( flags < 0 )
      {
        LOG_ERROR( "Failed to get client socket flags on port %u: %d", config.port, errno );
        return false;
      }

      if( ::fcntl( socket_context.client_fd, F_SETFL, flags | O_NONBLOCK ) < 0 )
      {
        LOG_ERROR( "Failed to set client socket non-blocking on port %u: %d", config.port, errno );
        return false;
      }

      return true;
    }

    bool handleRx()
    {
      if( socket_context.client_fd < 0 )
      {
        return false;
      }

      bool connection_ok = true;

      while( true )
      {
        const size_t bytes_needed = rx_buffer.size() - rx_bytes_pending;

        if( !bytes_needed )
        {
          break;
        }

        ssize_t bytes_read =
            ::recv( socket_context.client_fd, rx_buffer.data() + rx_bytes_pending, bytes_needed, MSG_DONTWAIT );

        if( bytes_read == 0 )
        {
          LOG_TRACE( "TCP client disconnected on port %u", config.port );
          connection_ok = false;
          break;
        }
        else if( bytes_read < 0 )
        {
          if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
          {
            break;
          }

          LOG_ERROR( "Socket recv failed on port %u: %d", config.port, errno );
          connection_ok = false;
          break;
        }

        rx_bytes_pending += static_cast<size_t>( bytes_read );

        // Process received data
        if( rx_callback && rx_bytes_pending > 0 )
        {
          std::scoped_lock lock( callback_mutex );
          rx_callback( *server_ref, rx_buffer.data(), rx_bytes_pending );
          rx_bytes_pending = 0;
        }
      }

      return connection_ok;
    }

    void handleTx()
    {
      if( socket_context.client_fd < 0 )
      {
        return;
      }

      // Get next packet from queue if none pending
      if( tx_bytes_pending == 0 )
      {
        std::scoped_lock lock( tx_mutex );
        if( !tx_queue.empty() )
        {
          tx_buffer = std::move( tx_queue.front() );
          tx_queue.pop();
          tx_bytes_pending = tx_buffer.size();
        }
      }

      if( tx_bytes_pending == 0 )
      {
        return;
      }

      const size_t  offset = tx_buffer.size() - tx_bytes_pending;
      const ssize_t bytes_sent =
          ::send( socket_context.client_fd, tx_buffer.data() + offset, tx_bytes_pending, MSG_NOSIGNAL | MSG_DONTWAIT );

      if( bytes_sent < 0 )
      {
        if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
        {
          return;
        }

        if( ( errno == EPIPE ) || ( errno == ECONNRESET ) )
        {
          LOG_TRACE( "TCP client disconnected during send on port %u", config.port );
        }
        else
        {
          LOG_ERROR( "Socket send failed on port %u: %d", config.port, errno );
        }

        cleanup();
        return;
      }

      tx_bytes_pending -= static_cast<size_t>( bytes_sent );
    }

    void cleanup()
    {
      const bool was_connected = client_connected.exchange( false );
      socket_context.closeAll();

      std::scoped_lock lock( tx_mutex );
      while( !tx_queue.empty() )
      {
        tx_queue.pop();
      }
      tx_bytes_pending = 0;

      if( was_connected && connection_callback )
      {
        std::scoped_lock cb_lock( callback_mutex );
        connection_callback( *server_ref, ConnectionState::Disconnected );
      }
    }
  };

  /*---------------------------------------------------------------------------
  Server Class Implementation
  ---------------------------------------------------------------------------*/

  Server::Server( const ServerConfig &config ) : m_impl( std::make_unique<Impl>( config, this ) )
  {
  }

  Server::~Server()
  {
    stop();
  }

  bool Server::start()
  {
    return m_impl->start();
  }

  void Server::stop()
  {
    m_impl->stop();
  }

  bool Server::sendData( const void *data, size_t size )
  {
    return m_impl->sendData( data, size );
  }

  bool Server::isRunning() const
  {
    return m_impl->isRunning();
  }

  bool Server::isClientConnected() const
  {
    return m_impl->isClientConnected();
  }

  uint16_t Server::getPort() const
  {
    return m_impl->getPort();
  }

  void Server::setRxCallback( DataReceivedCallback callback )
  {
    m_impl->setRxCallback( callback );
  }

  void Server::setConnectionCallback( ConnectionCallback callback )
  {
    m_impl->setConnectionCallback( callback );
  }

  /*---------------------------------------------------------------------------
  Server Manager Implementation
  ---------------------------------------------------------------------------*/

  ServerManager &ServerManager::getInstance()
  {
    static ServerManager instance;
    return instance;
  }

  std::shared_ptr<Server> ServerManager::createServer( const ServerConfig &config )
  {
    std::scoped_lock lock( m_mutex );

    // Check if server already exists on this port
    for( const auto &server : m_servers )
    {
      if( server->getPort() == config.port )
      {
        LOG_WARN( "TCP server already exists on port %u", config.port );
        return server;
      }
    }

    auto server = std::make_shared<Server>( config );
    if( server->start() )
    {
      m_servers.push_back( server );
      LOG_DEBUG( "Created server on port %u", config.port );
      return server;
    }

    LOG_ERROR( "Failed to create server on port %u", config.port );
    return nullptr;
  }

  std::shared_ptr<Server> ServerManager::getServer( uint16_t port )
  {
    std::scoped_lock lock( m_mutex );

    for( const auto &server : m_servers )
    {
      if( server->getPort() == port )
      {
        return server;
      }
    }

    return nullptr;
  }

  bool ServerManager::removeServer( uint16_t port )
  {
    std::scoped_lock lock( m_mutex );

    auto it = std::find_if( m_servers.begin(), m_servers.end(),
                            [ port ]( const std::shared_ptr<Server> &server ) { return server->getPort() == port; } );

    if( it != m_servers.end() )
    {
      ( *it )->stop();
      m_servers.erase( it );
      LOG_DEBUG( "Removed TCP server on port %u", port );
      return true;
    }

    return false;
  }

  std::vector<uint16_t> ServerManager::getActivePorts() const
  {
    std::scoped_lock lock( m_mutex );

    std::vector<uint16_t> ports;
    ports.reserve( m_servers.size() );

    for( const auto &server : m_servers )
    {
      ports.push_back( server->getPort() );
    }

    return ports;
  }


}    // namespace Orbit::Sim::TCP

#endif /* SIMULATOR */
