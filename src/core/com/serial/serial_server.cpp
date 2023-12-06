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
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_server.hpp>
#include <src/core/hw/orbit_usart.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  Chimera::Status_t sendAckNack( const bool ack_or_nack, const Header &header, const StatusCode code )
  {
    LOG_TRACE_IF( ack_or_nack == false, "NACK: %d", code );

    /*-------------------------------------------------------------------------
    Ship the response on the wire
    -------------------------------------------------------------------------*/
    Message::AckNack reply;
    reply.raw.header.msgId = MsgId_MSG_ACK_NACK;
    reply.raw.header.subId = header.subId;
    reply.raw.header.uuid  = header.uuid;
    reply.raw.acknowledge  = ack_or_nack;
    reply.raw.status_code  = code;


    if( Message::encode( &reply.state ) )
    {
      return Message::send( &reply.state, Config::getCommandPort() );
    }
    else
    {
      return Chimera::Status::ENCODE_ERROR;
    }
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  DispatchServer::DispatchServer() : mSerial( nullptr ), mRXBuffer( nullptr ), mLastValidMsg( 0 )
  {
  }


  DispatchServer::~DispatchServer()
  {
  }


  Chimera::Status_t DispatchServer::initialize( Chimera::Serial::Driver_rPtr    device,
                                                etl::icircular_buffer<uint8_t> &msg_buffer )
  {
    /*-------------------------------------------------------------------------
    Store the data with the class
    -------------------------------------------------------------------------*/
    mSerial   = device;
    mRXBuffer = &msg_buffer;
    RT_DBG_ASSERT( mSerial && mRXBuffer );

    /*-------------------------------------------------------------------------
    Initialize memory
    -------------------------------------------------------------------------*/
    mRXBuffer->clear();
    mLastValidMsg = Chimera::millis();

    return Chimera::Status::OK;
  }


  void DispatchServer::process()
  {
    /*-------------------------------------------------------------------------
    Empty the buffer first of any queued messages, then pull in new data
    -------------------------------------------------------------------------*/
    this->dispatchMessages();
    this->accumulateMessages();
  }


  /**
   * @brief Processes any queued messages in the internal buffer.
   *
   * Frames arrive as COBS encoded, nanopb serialized messages. This function
   * decodes all of the frames in the buffer and dispatches them to the
   * appropriate handler.
   */
  void DispatchServer::dispatchMessages()
  {
    bool         end_of_frame = false;
    const size_t current_time = Chimera::millis();

    /*-------------------------------------------------------------------------
    Process the receive buffer until we run out of data or frames
    -------------------------------------------------------------------------*/
    while( mRXBuffer->size() >= Message::MIN_COBS_MSG_SIZE )
    {
      /*-----------------------------------------------------------------------
      Search for the tail of a COBS message, indicated by a \x00 byte
      -----------------------------------------------------------------------*/
      size_t rx_search_offset = Message::MIN_COBS_MSG_SIZE;
      for( auto idx = mRXBuffer->begin() + rx_search_offset; ( ( end_of_frame == false ) && ( idx != mRXBuffer->end() ) );
           idx++, rx_search_offset++ )
      {
        end_of_frame = ( *idx == '\x00' );
      }

      if( !end_of_frame )
      {
        break;
      }

      /*-----------------------------------------------------------------------
      Copy the frame out into local contiguous working buffers
      -----------------------------------------------------------------------*/
      DecodeBuffer_t out, in;
      out.fill( 0 );
      in.fill( 0 );

      RT_DBG_ASSERT( in.size() >= rx_search_offset );
      for( size_t idx = 0; idx < rx_search_offset; idx++ )
      {
        in[ idx ] = mRXBuffer->front();
        mRXBuffer->pop();
      }

      RT_DBG_ASSERT( in[ rx_search_offset ] == '\x00' );

      /*-----------------------------------------------------------------------
      Attempt to decode the COBS frame
      -----------------------------------------------------------------------*/
      cobs_decode_result cobsResult = cobs_decode( out.data(), out.size(), in.cbegin(), rx_search_offset - 1u );
      if( cobsResult.status != COBS_DECODE_OK )
      {
        LOG_ERROR( "COBS decode failed: %d", cobsResult.status );
        continue;
      }

      /*-----------------------------------------------------------------------
      Decode the nanopb data from the COBS frame
      -----------------------------------------------------------------------*/
      BaseMessage  msg;
      pb_istream_t stream = pb_istream_from_buffer( reinterpret_cast<const pb_byte_t *>( out.data() ), cobsResult.out_len );

      if( !pb_decode( &stream, BaseMessage_fields, &msg ) )
      {
        LOG_ERROR( "Malformed protobuf message" );
        continue;
      }
      else
      {
        mLastValidMsg = current_time;
      }

      /*-----------------------------------------------------------------------
      Process the message
      -----------------------------------------------------------------------*/
      switch( msg.header.msgId )
      {
        case MsgId_MSG_PING_CMD: {
          Message::Ping msg;
          if( Message::decode( &msg.state, in.data(), rx_search_offset - 1u ) )
          {
            this->receive( msg );
          }
        }
        break;

        case MsgId_MSG_PARAM_IO: {
          Message::ParamIO msg;
          if( Message::decode( &msg.state, in.data(), rx_search_offset - 1u ) )
          {
            this->receive( msg );
          }
        }
        break;

        case MsgId_MSG_SYS_CTRL: {
          Message::SystemControl msg;
          if( Message::decode( &msg.state, in.data(), rx_search_offset - 1u ) )
          {
            this->receive( msg );
          }
        }
        break;

        default:
          LOG_ERROR( "Unhandled msg ID: %d", msg.header.msgId );
          break;
      }
    }

    /*-------------------------------------------------------------------------
    Clear the buffer if it's been too long since we've seen a valid message
    -------------------------------------------------------------------------*/
    const bool has_data = mRXBuffer->size() > 0;
    const bool timeout  = ( current_time - mLastValidMsg ) > ( 5 * Chimera::Thread::TIMEOUT_1S );

    if( !end_of_frame && has_data && timeout )
    {
      LOG_ERROR( "Clearing serial server RX buffer due to timeout" );
      mRXBuffer->clear();
      mLastValidMsg = current_time;
    }
  }


  /**
   * @brief Accumulates messages from the serial bus into the internal buffer.
   *
   * We aren't guaranteed to receive a full frame at a time, so this function
   * will pull as much data as possible from the serial driver and push it into
   * the internal buffer. The dispatchMessages() function will then process
   * the buffer as needed.
   */
  void DispatchServer::accumulateMessages()
  {
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
      if( act_read_size > 0 )
      {
        mRXBuffer->push( read_chunk, read_chunk + act_read_size );
      }

      /*-----------------------------------------------------------------------
      Update our notion of how much space we can read next time around
      -----------------------------------------------------------------------*/
      free_size = mRXBuffer->available();
    } while( ( free_size > 0 ) && ( act_read_size > 0 ) );
  }
}    // namespace Orbit::Serial
