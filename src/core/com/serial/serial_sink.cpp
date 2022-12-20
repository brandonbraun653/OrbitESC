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


  Aurora::Logging::Result EncodedLogSink::log( const Aurora::Logging::Level level, const void *const message, const size_t length )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Make sure we can actually log the data
    -------------------------------------------------------------------------*/
    if( mSerial == nullptr )
    {
      return Result::RESULT_FAIL_BAD_SINK;
    }
    else if ( level < logLevel )
    {
      return Result::RESULT_INVALID_LEVEL;
    }
    else if( !message || length == 0 )
    {
      return Result::RESULT_NO_MEM;
    }

    /*-------------------------------------------------------------------------
    Construct the encoded data
    -------------------------------------------------------------------------*/
    Message::Console msg;
    ConsoleMessage   pb_data;
    size_t           bytes_to_write = length;
    size_t           byte_offset    = 0;
    uint8_t          frame_number   = 0;
    const uint8_t   *p_usr_data     = reinterpret_cast<const uint8_t *>( message );

    while( bytes_to_write )
    {
      const uint8_t chunk_size = std::min<uint8_t>( sizeof( ConsoleMessage::data ), bytes_to_write );

      /*-----------------------------------------------------------------------
      Construct the message
      -----------------------------------------------------------------------*/
      memset( &pb_data, 0, sizeof( pb_data ) );
      msg.reset();

      pb_data.header.msgId = Message::Console::MessageId;
      pb_data.header.size  = chunk_size;
      pb_data.header.subId = 0;
      pb_data.frame        = frame_number++;
      memcpy( pb_data.data, p_usr_data + byte_offset, chunk_size );

      /*-----------------------------------------------------------------------
      Encode the data and ship it
      -----------------------------------------------------------------------*/
      if( !msg.encode( pb_data ) )
      {
        return Result::RESULT_FAIL;
      }

      if( msg.send( mSerial, true ) == Chimera::Status::OK )
      {
        byte_offset += chunk_size;
        bytes_to_write -= chunk_size;
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

}  // namespace Orbit::Serial