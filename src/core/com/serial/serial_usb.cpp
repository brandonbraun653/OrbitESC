/******************************************************************************
 *  File Name:
 *    serial_usb.cpp
 *
 *  Description:
 *    Tiny USB Serial Driver Implementation
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#if defined( EMBEDDED )

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/serial>
#include <Chimera/thread>
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/hw/orbit_usb.hpp>
#include <src/core/tasks.hpp>

#include <tusb.h>

namespace Orbit::Serial
{
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
    return tud_cdc_n_connected( Endpoint::COM_ENDPOINT );
  }


  Chimera::Serial::Driver_rPtr Config::getCommandPort()
  {
    return reinterpret_cast<Chimera::Serial::Driver_rPtr>( &s_usb_serial );
  }

  /*---------------------------------------------------------------------------
  USBSerial Implementation
  ---------------------------------------------------------------------------*/

  USBSerial::USBSerial() : mEndpoint( Endpoint::COM_ENDPOINT ), mRXBuffer( nullptr ), mTXBuffer( nullptr )
  {
  }


  USBSerial::~USBSerial()
  {
  }


  Chimera::Status_t USBSerial::init( const Endpoint endpoint, CircularBuffer prx, CircularBuffer ptx )
  {
    using namespace Chimera::Function;

    /*-------------------------------------------------------------------------
    Assign the configuration
    -------------------------------------------------------------------------*/
    mEndpoint = endpoint;
    mRXBuffer = prx;
    mTXBuffer = ptx;

    /*-------------------------------------------------------------------------
    Reset the buffers
    -------------------------------------------------------------------------*/
    mRXBuffer->clear();
    mTXBuffer->clear();

    /*-------------------------------------------------------------------------
    Register the RTX handlers for this instance
    -------------------------------------------------------------------------*/
    Orbit::USB::onCDCRXComplete( mEndpoint, Opaque::create<USBSerial, &USBSerial::on_rx_complete>( *this ) );
    Orbit::USB::onCDCTXComplete( mEndpoint, Opaque::create<USBSerial, &USBSerial::on_tx_complete>( *this ) );

    return Chimera::Status::OK;
  }


  void USBSerial::process()
  {
    // on_tx_complete();
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

    if( !this->try_lock_for( timeout ) )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    First attempt to push as much data directly into the CDC driver as possible
    -------------------------------------------------------------------------*/
    const size_t cdc_avail_size = tud_cdc_n_write_available( mEndpoint );
    const size_t cdc_write_size = std::min( length, cdc_avail_size );
    size_t  read_idx = 0;

    if( cdc_write_size )
    {
      read_idx += static_cast<ssize_t>( tud_cdc_n_write( mEndpoint, buffer, cdc_write_size ) );
    }

    /*-------------------------------------------------------------------------
    Enqueue any remaining data into the TX buffer
    -------------------------------------------------------------------------*/
    const size_t to_write = length - read_idx;
    if( to_write > 0 )
    {
      ssize_t q_write_size = std::min( to_write, mTXBuffer->available() );
      while( q_write_size > 0 )
      {
        mTXBuffer->push( static_cast<const uint8_t *>( buffer )[ read_idx++ ] );
        q_write_size--;
      }
    }

    this->unlock();
    return static_cast<int>( read_idx );
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

    if( !this->try_lock_for( timeout ) )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Read data into the user buffer
    -------------------------------------------------------------------------*/
    const size_t read_size = std::min( length, mRXBuffer->size() );

    size_t bytes_read = 0;
    while( bytes_read < read_size )
    {
      static_cast<uint8_t *>( buffer )[ bytes_read ] = mRXBuffer->front();
      mRXBuffer->pop();
      bytes_read++;
    }

    this->unlock();
    return static_cast<int>( bytes_read );
  }


  void USBSerial::on_rx_complete()
  {
    /*-------------------------------------------------------------------------
    Pull data out from the USB driver and push it into the RX buffer
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard lck( *this );

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
  }


  void USBSerial::on_tx_complete()
  {
    if( !tud_mounted() || !tud_cdc_n_connected( mEndpoint ) )
    {
      return;
    }

    if( !this->try_lock_for( Chimera::Thread::TIMEOUT_DONT_WAIT ) )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Process the ISR TX buffer first as it has priority. Usually this buffer is
    used for realtime data monitoring and needs to be serviced as quickly as
    possible.
    -------------------------------------------------------------------------*/
    // while( !mTXBufferISR->empty() )
    // {
    //   /*-----------------------------------------------------------------------
    //   Ensure there is data available to write and a place to put it
    //   -----------------------------------------------------------------------*/
    //   const size_t usb_bytes = tud_cdc_n_write_available( mEndpoint );
    //   const size_t buf_bytes = mTXBufferISR->size();

    //   if( !usb_bytes || !buf_bytes )
    //   {
    //     break;
    //   }

    //   /*-----------------------------------------------------------------------
    //   Write the data from the TX buffer into the USB driver. The buffer isn't
    //   guaranteed to be contiguous, so write byte by byte.
    //   -----------------------------------------------------------------------*/
    //   int write_size = static_cast<int>( std::min( usb_bytes, buf_bytes ) );

    //   while( write_size > 0 )
    //   {
    //     const uint32_t write_count = tud_cdc_n_write_char( mEndpoint, mTXBufferISR->front() );
    //     if( write_count == 1u )
    //     {
    //       mTXBufferISR->pop();
    //       write_size--;
    //     }
    //     else
    //     {
    //       break;
    //     }
    //   }
    // }

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

    this->unlock();
  }

}    // namespace Orbit::Serial

#endif /* EMBEDDED */
