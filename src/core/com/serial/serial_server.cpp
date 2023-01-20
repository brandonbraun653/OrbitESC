/******************************************************************************
 *  File Name:
 *    serial_server.cpp
 *
 *  Description:
 *    Serial server implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <Chimera/serial>
#include <src/core/com/serial/serial_server.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  DispatchServer::DispatchServer() : mSerial( nullptr ), mRXBuffer( nullptr )
  {
  }


  DispatchServer::~DispatchServer()
  {
  }


  Chimera::Status_t DispatchServer::initialize( const Chimera::Serial::Channel  channel,
                                                etl::icircular_buffer<uint8_t> &msg_buffer )
  {
    /*-------------------------------------------------------------------------
    Store the data with the class
    -------------------------------------------------------------------------*/
    mSerial   = Chimera::Serial::getDriver( channel );
    mRXBuffer = &msg_buffer;
    RT_DBG_ASSERT( mSerial && mRXBuffer );

    /*-------------------------------------------------------------------------
    Initialize memory
    -------------------------------------------------------------------------*/
    mRXBuffer->clear();

    return Chimera::Status::OK;
  }


  void DispatchServer::process()
  {
    /*-------------------------------------------------------------------------
    Empty the buffer first of any queued messages
    -------------------------------------------------------------------------*/
    this->dispatchMessages();

    /*-------------------------------------------------------------------------
    Pull as much data into the queue as possible
    -------------------------------------------------------------------------*/
    uint8_t read_chunk[ 64 ];
    size_t  act_read_size = 0;
    size_t  free_size     = mRXBuffer->available();

    do
    {
      /*-----------------------------------------------------------------------
      Decide how many bytes to read from the driver, then perform the read.
      -----------------------------------------------------------------------*/
      const size_t req_size = std::min<size_t>( free_size, ARRAY_BYTES( read_chunk ) );
      memset( read_chunk, 0, ARRAY_BYTES( read_chunk ) );
      act_read_size = mSerial->read( read_chunk, req_size );

      /*-----------------------------------------------------------------------
      Push into the buffer if we got data. Ignore overwrites. Higher level
      protocols should handle any side-effects of packet corruption.
      -----------------------------------------------------------------------*/
      if ( act_read_size > 0 )
      {
        mRXBuffer->push( read_chunk, read_chunk + act_read_size );
      }

      /*-----------------------------------------------------------------------
      Update our notion of how much space we can read next time around
      -----------------------------------------------------------------------*/
      free_size = mRXBuffer->available();
    } while ( ( free_size > 0 ) && ( act_read_size > 0 ) );
  }


  void DispatchServer::dispatchMessages()
  {
    /*-------------------------------------------------------------------------
    Parse the buffer for messages until none are found or the buffer is empty
    -------------------------------------------------------------------------*/
    while( mRXBuffer->size() > 0 )
    {
      /*-----------------------------------------------------------------------
      Search for the tail of a COBS message, indicated by a \x00 byte
      -----------------------------------------------------------------------*/
      // If buffer gets full, clear it. Issue console error message.


      /*-----------------------------------------------------------------------
      Decode the COBS message and inspect it for a supported header
      -----------------------------------------------------------------------*/
      // If decode fail: Issue console message
      // If success:
      //  Inspect header for supported message
      //  If supported, dispatch. Else issue console message about unsupported.
    }
  }

}    // namespace Orbit::Serial
