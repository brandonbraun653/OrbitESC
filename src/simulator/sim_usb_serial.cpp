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

#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr int PORT     = 37218;
  static constexpr int MAX_CONN = 1;

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


  Chimera::Status_t USBSerial::init( const size_t endpoint, CircularBuffer prx, CircularBuffer ptx, ISRLockedQueue ptx_isr )
  {
    /*-------------------------------------------------------------------------
    Assign the configuration
    -------------------------------------------------------------------------*/
    mEndpoint    = endpoint;
    mRXBuffer    = prx;
    mTXBuffer    = ptx;
    mTXBufferISR = ptx_isr;

    /*-------------------------------------------------------------------------
    Reset the buffers
    -------------------------------------------------------------------------*/
    mRXBuffer->clear();
    mTXBuffer->clear();
    mTXBufferISR->clear();

    /*-------------------------------------------------------------------------
    Initialize the server
    -------------------------------------------------------------------------*/
    s_server_config.server_fd = 0;
    s_server_config.client_fd = 0;
    s_server_config.opt       = 1;
    s_server_config.addrlen   = sizeof( s_server_config.address );
    s_server_config.connected = false;

    if( std::atexit( close_socket ) )
    {
      std::cerr << "Failed to register exit handler" << std::endl;
      return Chimera::Status::FAILED_INIT;
    }

    if( ( s_server_config.server_fd = socket( AF_INET, SOCK_STREAM, 0 ) ) == 0 )
    {
      std::cerr << "Failed to create socket" << std::endl;
      return Chimera::Status::FAILED_INIT;
    }

    if( setsockopt( s_server_config.server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &s_server_config.opt,
                    sizeof( s_server_config.opt ) ) )
    {
      std::cerr << "Failed to set socket options" << std::endl;
      return Chimera::Status::FAILED_INIT;
    }

    s_server_config.address.sin_family      = AF_INET;
    s_server_config.address.sin_addr.s_addr = INADDR_ANY;
    s_server_config.address.sin_port        = htons( PORT );

    if( bind( s_server_config.server_fd, ( struct sockaddr * )&s_server_config.address, sizeof( s_server_config.address ) ) <
        0 )
    {
      std::cerr << "Failed to bind socket" << std::endl;
      return Chimera::Status::FAILED_INIT;
    }

    if( listen( s_server_config.server_fd, MAX_CONN ) < 0 )
    {
      std::cerr << "Failed to listen on socket" << std::endl;
      return Chimera::Status::FAILED_INIT;
    }

    if( fcntl( s_server_config.server_fd, F_SETFL, O_NONBLOCK ) < 0 )
    {
      std::cerr << "Failed to configure the socket as non-blocking" << std::endl;
      return Chimera::Status::FAILED_INIT;
    }

    return Chimera::Status::OK;
  }


  void USBSerial::process()
  {
    /*-------------------------------------------------------------------------
    Establish the connection if it hasn't been done yet
    -------------------------------------------------------------------------*/
    if( !s_server_config.connected )
    {
      s_server_config.client_fd = accept( s_server_config.server_fd, ( struct sockaddr * )&s_server_config.address,
                                          ( socklen_t * )&s_server_config.addrlen );
      if( s_server_config.client_fd > 0 )
      {
        s_server_config.connected = true;
      }
      else
      {
        return;
      }
    }

    Chimera::Thread::LockGuard _lock( *this );

    /*-------------------------------------------------------------------------
    Process the RX buffer
    -------------------------------------------------------------------------*/
    while( !mRXBuffer->full() )
    {
      /*-----------------------------------------------------------------------
      Ensure there is data available to read and a place to put it
      -----------------------------------------------------------------------*/
      const size_t buf_bytes = mRXBuffer->available();
      if( !buf_bytes )
      {
        break;
      }

      uint8_t *input_buffer = new uint8_t[ buf_bytes ];
      RT_HARD_ASSERT( input_buffer );

      /*-----------------------------------------------------------------------
      Read the data from the socket and push it into the RX buffer
      -----------------------------------------------------------------------*/
      const ssize_t act_size = recv( s_server_config.client_fd, input_buffer, buf_bytes, 0 );
      if( act_size < 0 && errno != EAGAIN )
      {
        LOG_ERROR( "Failed to read from socket: %d", errno );
        delete[] input_buffer;
        break;
      }
      else if( act_size == 0 )
      {
        LOG_INFO( "Client disconnected" );
        ::close( s_server_config.client_fd );
        s_server_config.client_fd = -1;
        s_server_config.connected = false;

        delete[] input_buffer;

        /* Return here b/c the following "send" calls require a connection */
        return;
      }

      for( ssize_t i = 0; i < act_size; i++ )
      {
        mRXBuffer->push( input_buffer[ i ] );
      }

      delete[] input_buffer;
    }

    /*-------------------------------------------------------------------------
    Process the TX buffer
    -------------------------------------------------------------------------*/
    while( !mTXBuffer->empty() )
    {
      const size_t buf_bytes = mTXBuffer->size();
      if( !buf_bytes )
      {
        break;
      }

      uint8_t *output_buffer = new uint8_t[ buf_bytes ];
      RT_HARD_ASSERT( output_buffer );

      for( size_t i = 0; i < buf_bytes; i++ )
      {
        output_buffer[ i ] = mTXBuffer->front();
        mTXBuffer->pop();
      }

      const ssize_t act_sent = send( s_server_config.client_fd, output_buffer, buf_bytes, 0 );
      if( act_sent < 0 && errno != EAGAIN )
      {
        LOG_ERROR( "Failed to send to socket: %d", errno );
      }
      else if( act_sent != buf_bytes )
      {
        LOG_ERROR( "Failed to send all data to socket. Attempted: %d, Actual: %d", buf_bytes, act_sent );
      }

      delete[] output_buffer;
    }

    /*-------------------------------------------------------------------------
    Process the ISR TX buffer
    -------------------------------------------------------------------------*/
    while( !mTXBufferISR->empty() )
    {
      const size_t buf_bytes = mTXBufferISR->size();
      if( !buf_bytes )
      {
        break;
      }

      uint8_t *output_buffer = new uint8_t[ buf_bytes ];
      RT_HARD_ASSERT( output_buffer );

      for( size_t i = 0; i < buf_bytes; i++ )
      {
        output_buffer[ i ] = mTXBufferISR->front();
        mTXBufferISR->pop();
      }

      const ssize_t act_sent = send( s_server_config.client_fd, output_buffer, buf_bytes, 0 );
      if( act_sent < 0 && errno != EAGAIN )
      {
        LOG_ERROR( "Failed to send to socket: %d", errno );
      }
      else if( act_sent != buf_bytes )
      {
        LOG_ERROR( "Failed to send all data to socket. Attempted: %d, Actual: %d", buf_bytes, act_sent );
      }

      delete[] output_buffer;
    }
  }


  Chimera::Status_t USBSerial::open( const Chimera::Serial::Config &config )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t USBSerial::close()
  {
    return Chimera::Status::OK;
  }


  int USBSerial::write( const void *const buffer, const size_t length, const size_t timeout )
  {
    using namespace Orbit::Tasks;

    /*-------------------------------------------------------------------------
    Validate input arguments
    -------------------------------------------------------------------------*/
    if( !buffer || !length || !mTXBuffer || !s_server_config.connected )
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

    /*-------------------------------------------------------------------------
    Notify the CDC thread there is data to process
    -------------------------------------------------------------------------*/
    if( bytes_written > 0 )
    {
      Chimera::Thread::sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ), TASK_MSG_CDC_WAKEUP,
                                    Chimera::Thread::TIMEOUT_DONT_WAIT );
    }

    return static_cast<int>( bytes_written );
  }


  int USBSerial::writeFromISR( const void *const buffer, const size_t length )
  {
    using namespace Orbit::Tasks;

    /*-------------------------------------------------------------------------
    Validate input arguments
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( buffer );
    RT_DBG_ASSERT( length );
    RT_DBG_ASSERT( mTXBufferISR );

    /*-------------------------------------------------------------------------
    Enqueue the data into the TX buffer
    -------------------------------------------------------------------------*/
    if( length <= mTXBufferISR->available_from_unlocked() )
    {
      size_t bytes_written = 0;
      while( bytes_written < length )
      {
        mTXBufferISR->push_from_unlocked( static_cast<const uint8_t *const>( buffer )[ bytes_written++ ] );
      }

      Chimera::Thread::sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ), TASK_MSG_CDC_WAKEUP,
                                    Chimera::Thread::TIMEOUT_DONT_WAIT );

      return static_cast<int>( length );
    }

    return 0;
  }


  int USBSerial::read( void *const buffer, const size_t length, const size_t timeout )
  {
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

}    // namespace Orbit::Serial

#endif /* SIMULATOR */
