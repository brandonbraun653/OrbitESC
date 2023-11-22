/******************************************************************************
 *  File Name:
 *    serial_async_message.cpp
 *
 *  Description:
 *    Utility functions for serial messages
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/atomic.h>
#include <src/core/com/serial/serial_async_message.hpp>

namespace Orbit::Serial::Message
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile etl::atomic<UniqueId_t> s_msg_uuid;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void moduleInit()
  {
    s_msg_uuid = 0;
  }


  UniqueId_t getNextUUID()
  {
    return s_msg_uuid.fetch_add( 1, etl::memory_order_seq_cst );
  }


  bool decode( EncodableMessage *const msg, const void *const src, const size_t length )
  {
    /*-----------------------------------------------------------------------
    Input Protection
    -----------------------------------------------------------------------*/
    if( !msg || !src || ( length > msg->IOBufferSize ) )
    {
      RT_DBG_ASSERT( false );
      return false;
    }

    /*-----------------------------------------------------------------------
    Unpack the COBS encoded frame
    -----------------------------------------------------------------------*/
    cobs_decode_result cobsResult = cobs_decode( msg->IOBuffer, msg->IOBufferSize, src, length );
    if( cobsResult.status != COBS_DECODE_OK )
    {
      return false;
    }

    /*-----------------------------------------------------------------------
    Decode the nanopb data
    -----------------------------------------------------------------------*/
    pb_istream_t stream = pb_istream_from_buffer( msg->IOBuffer, cobsResult.out_len );
    return pb_decode( &stream, msg->PBPayloadFields, msg->PBPayloadData );
  }


  bool encode( EncodableMessage *const msg )
  {
    RT_DBG_ASSERT( msg );
    return encode( msg, msg->PBPayloadData, msg->PBPayloadSize );
  }


  bool encode( EncodableMessage *const msg, const void *const src, const size_t length )
  {
    RT_DBG_ASSERT( msg );
    RT_DBG_ASSERT( src );

    /*-------------------------------------------------------------------------
    Allocate some memory on the heap for the encoded message. Use the COBS
    message size as an upper limit on the size of the nanopb encoded message.
    Increase to the next power of two to avoid heap fragmentation.
    -------------------------------------------------------------------------*/
    static constexpr size_t ALLOC_SIZE = 256;
    static_assert( MAX_COBS_MSG_SIZE <= ALLOC_SIZE );
    pb_byte_t *PBBuffer = new pb_byte_t[ ALLOC_SIZE ];
    RT_HARD_ASSERT( PBBuffer );

    /*-----------------------------------------------------------------------
    Reset the working memory
    -----------------------------------------------------------------------*/
    memset( msg->IOBuffer, 0, msg->IOBufferSize );
    memset( PBBuffer, 0, ALLOC_SIZE );

    /*-----------------------------------------------------------------------
    Encode the data type using nanopb
    -----------------------------------------------------------------------*/
    pb_ostream_t stream    = pb_ostream_from_buffer( PBBuffer, ALLOC_SIZE );
    bool         pbSuccess = pb_encode( &stream, msg->PBPayloadFields, src );

    /*-----------------------------------------------------------------------
    Frame the transaction with COBS encoding. Add +1 to the total encoded
    size to account for the null byte delimiter that gets transmitted.
    -----------------------------------------------------------------------*/
    cobs_encode_result cobsResult = cobs_encode( msg->IOBuffer, msg->IOBufferSize, PBBuffer, stream.bytes_written );
    msg->EncodedSize              = cobsResult.out_len + 1u;

    RT_DBG_ASSERT( msg->EncodedSize <= msg->IOBufferSize );
    delete[] PBBuffer;
    return ( pbSuccess && ( cobsResult.status == COBS_ENCODE_OK ) );
  }


  Chimera::Status_t send( const EncodableMessage *const msg, Chimera::Serial::Driver_rPtr serial, const bool block )
  {
    using namespace Chimera::Event;
    using namespace Chimera::Thread;
    RT_DBG_ASSERT( serial );
    RT_DBG_ASSERT( msg );

    /*-----------------------------------------------------------------------
    Input Protection
    -----------------------------------------------------------------------*/
    if( ( msg->EncodedSize == 0 ) || ( msg->EncodedSize > msg->IOBufferSize ) )
    {
      return Chimera::Status::FAIL;
    }

    /*-----------------------------------------------------------------------
    Ship the data through the port
    -----------------------------------------------------------------------*/
    auto   timeout   = block ? TIMEOUT_BLOCK : TIMEOUT_DONT_WAIT;
    size_t offset    = 0;
    size_t remaining = msg->EncodedSize;

    while( offset < msg->EncodedSize )
    {
      const size_t n = serial->write( msg->IOBuffer + offset, remaining, timeout );
      if( n == 0 )
      {
        break;
      }

      offset += n;
      remaining -= n;

      RT_DBG_ASSERT( remaining <= msg->EncodedSize );
    }

    return ( offset == msg->EncodedSize ) ? Chimera::Status::OK : Chimera::Status::FAIL;
  }
}    // namespace Orbit::Serial::Message
