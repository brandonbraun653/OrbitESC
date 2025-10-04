/******************************************************************************
 *  File Name:
 *    sim_usb_serial.cpp
 *
 *  Description:
 *    Socket based implementation of the USB serial interface. This runs a very
 *    simple server that listens for incoming connections and then processes
 *    the data as it comes in.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#if defined( SIMULATOR )

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/serial>
#include <Chimera/thread>
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/tasks.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr int  PORT       = 37218;
  static constexpr int  MAX_CONN   = 1;
  static constexpr bool DEBUG_INFO = false;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ServerConfig
  {
    int                server_fd;
    int                client_fd;
    struct sockaddr_in address;
    int                opt;
    int                addrlen;
    bool               connected;
  };


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static USBSerial    s_usb_serial;
  static ServerConfig s_server_config;


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Ensures any open file descriptors are closed when the program exits
   * @return void
   */
  static void close_socket()
  {
    if( s_server_config.client_fd > 0 )
    {
      ::close( s_server_config.client_fd );
      s_server_config.client_fd = -1;
    }

    if( s_server_config.server_fd > 0 )
    {
      ::close( s_server_config.server_fd );
      s_server_config.server_fd = -1;
    }

    s_server_config.connected = false;
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  USBSerial *getUSBSerialDriver()
  {
    return &s_usb_serial;
  }


  bool isConnected()
  {
    return s_server_config.connected;
  }


  Chimera::Serial::Driver_rPtr Config::getCommandPort()
  {
    return reinterpret_cast<Chimera::Serial::Driver_rPtr>( &s_usb_serial );
  }

  /*---------------------------------------------------------------------------
  USBSerial Implementation
  ---------------------------------------------------------------------------*/

  USBSerial::USBSerial() : mEndpoint( 0 ), mRXBuffer( nullptr ), mTXBuffer( nullptr )
  {
  }


  USBSerial::~USBSerial()
  {
  }


  Chimera::Status_t USBSerial::init( const Endpoint endpoint, CircularBuffer prx, CircularBuffer ptx )
  {
    /*-------------------------------------------------------------------------
    Assign the configuration
    -------------------------------------------------------------------------*/
    mEndpoint = endpoint;
    mRXBuffer = prx;
    mTXBuffer = ptx;

    /*-------------------------------------------------------------------------
    Reset the buffers
    -------------------------------------------------------------------------*/
    if( mRXBuffer )
    {
      mRXBuffer->clear();
    }

    if( mTXBuffer )
    {
      mTXBuffer->clear();
    }

    /*-------------------------------------------------------------------------
    Initialize the server
    -------------------------------------------------------------------------*/
    s_server_config.server_fd = -1;
    s_server_config.client_fd = -1;
    s_server_config.opt       = 1;
    s_server_config.addrlen   = sizeof( s_server_config.address );
    s_server_config.connected = false;

    if( std::atexit( close_socket ) )
    {
      LOG_ERROR( "Failed to register exit handler" );
      return Chimera::Status::FAILED_INIT;
    }

    return Chimera::Status::OK;
  }


  void USBSerial::process()
  {
    Chimera::Thread::LockGuard _lock( *this );

    if( !mRXBuffer || !mTXBuffer )
    {
      return;
    }

    if( s_server_config.server_fd < 0 )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Establish the connection if it hasn't been done yet
    -------------------------------------------------------------------------*/
    if( !s_server_config.connected )
    {
      const int client_fd = accept( s_server_config.server_fd, reinterpret_cast<struct sockaddr *>( &s_server_config.address ),
                                    reinterpret_cast<socklen_t *>( &s_server_config.addrlen ) );

      if( client_fd >= 0 )
      {
        s_server_config.client_fd = client_fd;

        int flags = fcntl( s_server_config.client_fd, F_GETFL, 0 );
        if( flags >= 0 )
        {
          if( fcntl( s_server_config.client_fd, F_SETFL, flags | O_NONBLOCK ) < 0 )
          {
            LOG_ERROR( "Failed to set client socket non-blocking: %d", errno );
          }
        }
        else
        {
          LOG_ERROR( "Failed to get client socket flags: %d", errno );
        }

        s_server_config.connected = true;
        LOG_DEBUG_IF( DEBUG_INFO, "Client connected" );
      }
      else
      {
        if( ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
        {
          LOG_ERROR( "Failed to accept client connection: %d", errno );
        }

        return;
      }
    }

    if( !s_server_config.connected )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Process the RX buffer
    -------------------------------------------------------------------------*/
    if( !mRXBuffer->full() )
    {
      const size_t buf_bytes = mRXBuffer->available();

      if( buf_bytes > 0 )
      {
        std::vector<uint8_t> input_buffer( buf_bytes );
        const ssize_t        read_size = recv( s_server_config.client_fd, input_buffer.data(), buf_bytes, MSG_DONTWAIT );

        if( read_size > 0 )
        {
          for( ssize_t i = 0; i < read_size; ++i )
          {
            mRXBuffer->push( input_buffer[ static_cast<size_t>( i ) ] );
          }
        }
        else if( read_size == 0 )
        {
          LOG_DEBUG_IF( DEBUG_INFO, "Client disconnected" );
          close_socket();

          if( mRXBuffer )
          {
            mRXBuffer->clear();
          }

          if( mTXBuffer )
          {
            mTXBuffer->clear();
          }

          return;
        }
        else if( ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
        {
          LOG_ERROR( "Failed to read from socket: %d", errno );
        }
      }
    }

    /*-------------------------------------------------------------------------
    Process the TX buffer
    -------------------------------------------------------------------------*/
    if( !mTXBuffer->empty() )
    {
      const size_t buf_bytes = mTXBuffer->size();

      if( buf_bytes > 0 )
      {
        std::vector<uint8_t> output_buffer( buf_bytes );

        for( size_t i = 0; i < buf_bytes; ++i )
        {
          output_buffer[ i ] = mTXBuffer->front();
          mTXBuffer->pop();
        }

        const ssize_t act_sent = send( s_server_config.client_fd, output_buffer.data(), buf_bytes, MSG_NOSIGNAL );

        if( act_sent < 0 )
        {
          if( ( errno == EPIPE ) || ( errno == ECONNRESET ) )
          {
            LOG_DEBUG_IF( DEBUG_INFO, "Client disconnected during send" );
            close_socket();
          }
          else if( ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
          {
            LOG_ERROR( "Failed to send to socket: %d", errno );
          }
        }
        else if( static_cast<size_t>( act_sent ) != buf_bytes )
        {
          LOG_ERROR( "Failed to send all data to socket. Attempted: %d, Actual: %d", buf_bytes, act_sent );
        }
      }
    }
  }


  Chimera::Status_t USBSerial::open( const Chimera::Serial::Config &config )
  {
    static_cast<void>( config );

    RT_DBG_ASSERT( s_server_config.server_fd == -1 );
    RT_DBG_ASSERT( s_server_config.client_fd == -1 );

    /*-------------------------------------------------------------------------
    Create the socket
    -------------------------------------------------------------------------*/
    if( ( s_server_config.server_fd = socket( AF_INET, SOCK_STREAM, 0 ) ) == 0 )
    {
      LOG_ERROR( "Failed to create socket" );
      close_socket();
      return Chimera::Status::FAILED_INIT;
    }

    /*-------------------------------------------------------------------------
    Set the socket options
    -------------------------------------------------------------------------*/
    if( setsockopt( s_server_config.server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &s_server_config.opt,
                    sizeof( s_server_config.opt ) ) )
    {
      LOG_ERROR( "Failed to set socket options" );
      close_socket();
      return Chimera::Status::FAILED_INIT;
    }

    /*-------------------------------------------------------------------------
    Bind the socket to the desired port
    -------------------------------------------------------------------------*/
    s_server_config.address.sin_family      = AF_INET;
    s_server_config.address.sin_addr.s_addr = INADDR_ANY;
    s_server_config.address.sin_port        = htons( PORT );

    if( bind( s_server_config.server_fd, ( struct sockaddr * )&s_server_config.address, sizeof( s_server_config.address ) ) <
        0 )
    {
      LOG_ERROR( "Failed to bind socket" );
      close_socket();
      return Chimera::Status::FAILED_INIT;
    }

    /*-------------------------------------------------------------------------
    Start listening for incoming connections
    -------------------------------------------------------------------------*/
    if( listen( s_server_config.server_fd, MAX_CONN ) < 0 )
    {
      LOG_ERROR( "Failed to listen on socket" );
      close_socket();
      return Chimera::Status::FAILED_INIT;
    }

    /*-------------------------------------------------------------------------
    Set the socket to non-blocking
    -------------------------------------------------------------------------*/
    if( fcntl( s_server_config.server_fd, F_SETFL, O_NONBLOCK ) < 0 )
    {
      LOG_ERROR( "Failed to configure the socket as non-blocking" );
      close_socket();
      return Chimera::Status::FAILED_INIT;
    }

    LOG_INFO( "Server initialized and listening on port %d", PORT );
    return Chimera::Status::OK;
  }


  Chimera::Status_t USBSerial::close()
  {
    close_socket();
    return Chimera::Status::OK;
  }


  size_t USBSerial::availableForWrite()
  {
    Chimera::Thread::LockGuard _lock( *this );

    return mTXBuffer ? mTXBuffer->available() : 0;
  }


  int USBSerial::write( const void *const buffer, const size_t length, const size_t timeout )
  {
    using namespace Orbit::Tasks;

    static_cast<void>( timeout );

    /*-------------------------------------------------------------------------
    Validate input arguments
    -------------------------------------------------------------------------*/
    if( !buffer || !length || !mTXBuffer )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Enqueue the data into the TX buffer
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lock( *this );

    const size_t buf_bytes  = mTXBuffer->available();
    const size_t write_size = std::min( length, buf_bytes );

    size_t bytes_written = 0;
    while( bytes_written < write_size )
    {
      mTXBuffer->push( static_cast<const uint8_t *>( buffer )[ bytes_written ] );
      bytes_written++;
    }

    return static_cast<int>( bytes_written );
  }


  int USBSerial::read( void *const buffer, const size_t length, const size_t timeout )
  {
    static_cast<void>( timeout );

    /*-------------------------------------------------------------------------
    Validate input arguments
    -------------------------------------------------------------------------*/
    if( !buffer || !length || !mRXBuffer )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Read data into the user buffer
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lock( *this );

    const size_t buf_bytes = mRXBuffer->size();
    const size_t read_size = std::min( length, buf_bytes );

    size_t bytes_read = 0;
    while( bytes_read < read_size )
    {
      static_cast<uint8_t *>( buffer )[ bytes_read ] = mRXBuffer->front();
      mRXBuffer->pop();
      bytes_read++;
    }

    return static_cast<int>( bytes_read );
  }


  void USBSerial::on_rx_complete()
  {
    /* No-op in the simulator build. */
  }


  void USBSerial::on_tx_complete()
  {
    /* No-op in the simulator build. */
  }

}    // namespace Orbit::Serial

#endif /* SIMULATOR */
