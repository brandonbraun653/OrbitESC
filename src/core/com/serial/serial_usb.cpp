/******************************************************************************
 *  File Name:
 *    serial_usb.cpp
 *
 *  Description:
 *    Tiny USB Serial Driver Implementation
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/serial>
#include <Chimera/thread>
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/tasks.hpp>
#include <tusb.h>

namespace Orbit::Serial
{

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr uint8_t USB_SERIAL_ENDPOINT = 0;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static USBSerial s_usb_serial;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  USBSerial *getUSBSerialDriver()
  {
    return &s_usb_serial;
  }


  bool isConnected()
  {
    return tud_cdc_n_connected( USB_SERIAL_ENDPOINT );
  }


  Chimera::Serial::Driver_rPtr Config::getCommandPort()
  {
    return reinterpret_cast<Chimera::Serial::Driver_rPtr>( &s_usb_serial );
  }

  /*---------------------------------------------------------------------------
  USBSerial Implementation
  ---------------------------------------------------------------------------*/

  USBSerial::USBSerial() : mEndpoint( USB_SERIAL_ENDPOINT ), mRXBuffer( nullptr ), mTXBuffer( nullptr )
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

    return Chimera::Status::OK;
  }


  void USBSerial::process()
  {
    Chimera::Thread::LockGuard _lock( *this );

    /*-------------------------------------------------------------------------
    Pull data out from the USB driver and push it into the RX buffer
    -------------------------------------------------------------------------*/
    while( !mRXBuffer->full() )
    {
      /*-----------------------------------------------------------------------
      Ensure there is data available to read and a place to put it
      -----------------------------------------------------------------------*/
      const size_t usb_bytes = tud_cdc_n_available( mEndpoint );
      const size_t buf_bytes = mRXBuffer->available();

      if( !usb_bytes || !buf_bytes )
      {
        break;
      }

      /*-----------------------------------------------------------------------
      Read the data from the USB driver and push it into the RX buffer. The
      buffer isn't guaranteed to be contiguous, so read byte by byte.
      -----------------------------------------------------------------------*/
      int read_size = static_cast<int>( std::min( usb_bytes, buf_bytes ) );

      while( read_size > 0 )
      {
        const int32_t byte = tud_cdc_n_read_char( mEndpoint );
        if( byte >= 0 )
        {
          mRXBuffer->push( static_cast<uint8_t>( byte ) );
          read_size--;
        }
        else
        {
          break;
        }
      }
    }

    /*-------------------------------------------------------------------------
    Process the normal TX buffer second. It only gets processed if the ISR
    hasn't filled the CDC write FIFO.
    -------------------------------------------------------------------------*/
    while( !mTXBuffer->empty() )
    {
      /*-----------------------------------------------------------------------
      Ensure there is data available to write and a place to put it
      -----------------------------------------------------------------------*/
      const size_t usb_bytes = tud_cdc_n_write_available( mEndpoint );
      const size_t buf_bytes = mTXBuffer->size();

      if( !usb_bytes || !buf_bytes )
      {
        break;
      }

      /*-----------------------------------------------------------------------
      Write the data from the TX buffer into the USB driver. The buffer isn't
      guaranteed to be contiguous, so write byte by byte.
      -----------------------------------------------------------------------*/
      int write_size = static_cast<int>( std::min( usb_bytes, buf_bytes ) );

      while( write_size > 0 )
      {
        const uint32_t write_count = tud_cdc_n_write_char( mEndpoint, mTXBuffer->front() );
        if( write_count == 1u )
        {
          mTXBuffer->pop();
          write_size--;
        }
        else
        {
          break;
        }
      }
    }

    /*-------------------------------------------------------------------------
    Process the ISR TX buffer first as it has priority. Usually this buffer is
    used for realtime data monitoring and needs to be serviced as quickly as
    possible.
    -------------------------------------------------------------------------*/
    size_t processed_bytes = 0;
    while( ( processed_bytes <= mTXBufferISR->max_size() ) && !mTXBufferISR->empty() )
    {
      /*-----------------------------------------------------------------------
      Ensure there is data available to write and a place to put it
      -----------------------------------------------------------------------*/
      const size_t usb_bytes = tud_cdc_n_write_available( mEndpoint );
      const size_t buf_bytes = mTXBufferISR->size();

      if( !usb_bytes || !buf_bytes )
      {
        break;
      }

      /*-----------------------------------------------------------------------
      Write the data from the TX buffer into the USB driver. The buffer isn't
      guaranteed to be contiguous, so write byte by byte.
      -----------------------------------------------------------------------*/
      int write_size = static_cast<int>( std::min( usb_bytes, buf_bytes ) );

      while( write_size > 0 )
      {
        const uint32_t write_count = tud_cdc_n_write_char( mEndpoint, mTXBufferISR->front() );
        if( write_count == 1u )
        {
          mTXBufferISR->pop();
          write_size--;
          processed_bytes++;
        }
        else
        {
          break;
        }
      }
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
    if( !buffer || !length || !mTXBuffer )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Make sure the device is connected before doing anything
    -------------------------------------------------------------------------*/
    if( !tud_cdc_n_connected( mEndpoint ) )
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

    -------------------------------------------------------------------------*/
    if( bytes_written > 0 )
    {
      Chimera::Thread::sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ),
                                    TASK_MSG_CDC_WAKEUP,
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

      Chimera::Thread::sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ),
                                    TASK_MSG_CDC_WAKEUP,
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
