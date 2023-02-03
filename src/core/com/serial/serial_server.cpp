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
#include <Aurora/logging>
#include <Chimera/assert>
#include <Chimera/serial>
#include <src/core/com/serial/serial_server.hpp>
#include <src/core/hw/orbit_usart.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t sendAckNack( const bool ack_or_nack, const Header &header )
  {
    /*-------------------------------------------------------------------------
    Populate the core message type
    -------------------------------------------------------------------------*/
    AckNackMessage payload;
    memset( &payload, 0, sizeof( payload ) );
    payload.header.msgId = Message::MSG_ACK_NACK;
    payload.header.subId = header.subId;
    payload.header.uuid  = header.uuid;
    payload.acknowledge  = ack_or_nack;

    /*-------------------------------------------------------------------------
    Ship the response on the wire
    -------------------------------------------------------------------------*/
    Message::AckNack reply;
    reply.reset();

    if ( reply.encode( payload ) )
    {
      return reply.send( Orbit::USART::SerialDriver );
    }
    else
    {
      return Chimera::Status::ENCODE_ERROR;
    }
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  DispatchServer::DispatchServer() : mSerial( nullptr ), mRXBuffer( nullptr ), mRXSearchOfst( 0 ), mLastTick( 0 )
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
    mSerial       = Chimera::Serial::getDriver( channel );
    mRXBuffer     = &msg_buffer;
    mRXSearchOfst = 0;
    RT_DBG_ASSERT( mSerial && mRXBuffer );

    /*-------------------------------------------------------------------------
    Initialize memory
    -------------------------------------------------------------------------*/
    mRXBuffer->clear();
    mLastTick = Chimera::millis();

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

    /*-------------------------------------------------------------------------
    Ship the system tick message as needed
    -------------------------------------------------------------------------*/
    size_t current_tick = Chimera::millis();
    if ( ( current_tick - mLastTick ) >= 1000 )
    {
      Message::SysTick tick;
      tick.reset();
      tick.payload.header.msgId = Message::MSG_SYS_TICK;
      tick.payload.tick         = current_tick;

      tick.encode();
      tick.send( mSerial );

      mLastTick = current_tick;
    }
  }


  void DispatchServer::dispatchMessages()
  {
    /*-------------------------------------------------------------------------
    Parse the buffer for messages until none are found or the buffer is empty
    -------------------------------------------------------------------------*/
    bool eof      = false;
    mRXSearchOfst = 0;

    /*-----------------------------------------------------------------------
    Search for the tail of a COBS message, indicated by a \x00 byte
    -----------------------------------------------------------------------*/
    for ( auto idx = mRXBuffer->begin() + mRXSearchOfst; ( ( eof == false ) && ( idx != mRXBuffer->end() ) );
          idx++, mRXSearchOfst++ )
    {
      RT_DBG_ASSERT( mRXSearchOfst <= mRXBuffer->max_size() );
      eof = ( *idx == '\x00' );
    }

    /*-----------------------------------------------------------------------
    Clear the buffer if no message is found and either:
      a) Buffer is full
      b) Doesn't meet minimum message size requirements
    -----------------------------------------------------------------------*/
    if ( ( eof == false ) && ( ( mRXSearchOfst == mRXBuffer->max_size() ) || ( mRXSearchOfst < Message::MIN_COBS_MSG_SIZE ) ) )
    {
      mRXBuffer->clear();
      mRXSearchOfst = 0;
      return;
    }

    /*-----------------------------------------------------------------------
    Decode the COBS message and inspect it for a supported header
    -----------------------------------------------------------------------*/
    if ( eof )
    {
      /*---------------------------------------------------------------------
      Copy the frame out into local contiguous working buffers
      ---------------------------------------------------------------------*/
      DecodeBuffer_t out, in;
      out.fill( 0 );
      in.fill( 0 );

      RT_DBG_ASSERT( in.size() >= mRXSearchOfst );
      size_t copy_bytes = 0;
      while ( mRXBuffer->size() )
      {
        in[ copy_bytes ] = mRXBuffer->front();
        mRXBuffer->pop();
        copy_bytes++;
      }

      /*---------------------------------------------------------------------
      Attempt to decode the frame
      ---------------------------------------------------------------------*/
      RT_DBG_ASSERT( in[ mRXSearchOfst ] == '\x00' );

      cobs_decode_result cobsResult = cobs_decode( out.data(), out.size(), in.cbegin(), mRXSearchOfst - 1u );
      if ( cobsResult.status == COBS_DECODE_OK )
      {
        /*-----------------------------------------------------------------------
        Decode the nanopb data
        -----------------------------------------------------------------------*/
        BaseMessage  msg;
        pb_istream_t stream = pb_istream_from_buffer( reinterpret_cast<const pb_byte_t *>( out.data() ), cobsResult.out_len );

        if ( pb_decode( &stream, BaseMessage_fields, &msg ) )
        {
          switch ( msg.header.msgId )
          {
            case Message::MSG_PING_CMD: {
              Message::Ping ping;
              ping.decode( in.data(), mRXSearchOfst - 1u );
              this->receive( ping );
            }
            break;

            case Message::MSG_PARAM_IO: {
              Message::ParamIO param;
              param.decode( in.data(), mRXSearchOfst - 1u );
              this->receive( param );
            }
            break;

            case Message::MSG_ID_COUNT:
            default:
              LOG_ERROR( "Unhandled msg ID: %d", msg.header.msgId );
              break;
          }
        }
      }
    }
  }

}    // namespace Orbit::Serial
