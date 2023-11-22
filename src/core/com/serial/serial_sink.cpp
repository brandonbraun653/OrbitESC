/******************************************************************************
 *  File Name:
 *    serial_sink.cpp
 *
 *  Description:
 *    Serial sink driver using COBS & nanopb encoded messages
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/event>
#include <Chimera/serial>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_sink.hpp>

namespace Orbit::Serial
{
  using namespace Aurora::Logging;

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  EncodedLogSink::EncodedLogSink() : mSerial( nullptr )
  {
  }


  EncodedLogSink::~EncodedLogSink()
  {
  }


  void EncodedLogSink::assignChannel( Chimera::Serial::Channel channel )
  {
    mSerial = Chimera::Serial::getDriver( channel );
    RT_DBG_ASSERT( mSerial );
  }


  Aurora::Logging::Result EncodedLogSink::open()
  {
    RT_DBG_ASSERT( mSerial );
    return Result::RESULT_SUCCESS;
  }


  Aurora::Logging::Result EncodedLogSink::close()
  {
    mSerial = nullptr;
    return Result::RESULT_SUCCESS;
  }


  Aurora::Logging::Result EncodedLogSink::flush()
  {
    return Result::RESULT_SUCCESS;
  }


  Aurora::Logging::IOType EncodedLogSink::getIOType()
  {
    return IOType::SERIAL_SINK;
  }


  Aurora::Logging::Result EncodedLogSink::log( const Aurora::Logging::Level level, const void *const message,
                                               const size_t length )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Make sure we can actually log the data
    -------------------------------------------------------------------------*/
    if ( mSerial == nullptr )
    {
      return Result::RESULT_FAIL_BAD_SINK;
    }
    else if ( level < logLevel )
    {
      return Result::RESULT_INVALID_LEVEL;
    }
    else if ( !message || length == 0 )
    {
      return Result::RESULT_NO_MEM;
    }

    /*-------------------------------------------------------------------------
    Construct the encoded data
    -------------------------------------------------------------------------*/
    Message::Console    msg;
    Message::UniqueId_t msg_uuid     = Message::getNextUUID();
    size_t              byte_offset  = 0;
    uint8_t             frame_number = 0;
    const uint8_t       max_frames   = ( length / sizeof( ConsoleMessage::data ) ) + 1u;
    const uint8_t      *p_usr_data   = reinterpret_cast<const uint8_t *>( message );

    while ( frame_number < max_frames)
    {
      const size_t remaining  = length - byte_offset;
      const size_t chunk_size = std::min<size_t>( sizeof( ConsoleMessage::data.bytes ), remaining );

      /*-----------------------------------------------------------------------
      Construct the message
      -----------------------------------------------------------------------*/
      msg.payload.header.msgId = Message::Console::MessageId;
      msg.payload.header.subId = 0;
      msg.payload.header.uuid  = msg_uuid;
      msg.payload.this_frame   = frame_number;
      msg.payload.total_frames = max_frames;
      msg.payload.data.size    = chunk_size;
      memcpy( msg.payload.data.bytes, p_usr_data + byte_offset, chunk_size );

      /*-----------------------------------------------------------------------
      Encode the data and ship it
      -----------------------------------------------------------------------*/
      if ( !Serial::Message::encode( &msg.state ) )
      {
        return Result::RESULT_FAIL;
      }

      if ( Serial::Message::send( &msg.state, mSerial, true ) == Chimera::Status::OK )
      {
        byte_offset += chunk_size;
        frame_number++;
      }
      else
      {
        return Result::RESULT_FAIL;
      }
    }

    /*-------------------------------------------------------------------------
    Reaching here means all the data transmitted ok
    -------------------------------------------------------------------------*/
    return Result::RESULT_SUCCESS;
  }

}    // namespace Orbit::Serial
