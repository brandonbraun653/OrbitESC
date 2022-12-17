/******************************************************************************
 *  File Name:
 *    serial_message_intf.hpp
 *
 *  Description:
 *    Common serial message interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_MESSAGE_INTF_HPP
#define ORBIT_SERIAL_MESSAGE_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "cobs.h"
#include "serial_interface.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <Chimera/common>
#include <Chimera/serial>
#include <etl/message.h>
#include <etl/array.h>

namespace Orbit::Serial::Message
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum Id : etl::message_id_t
  {
    MSG_PING_CMD = 1, /**< Simple PING to see if the node is alive */

  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Helper class to share common operations on custom messages
   *
   * @tparam MSG_ID         Field from Orbit::Serial::Message::Id
   * @tparam PayloadType    Structure type generated from proto file
   * @tparam PayloadSize    Maximum size of the encoded message
   * @tparam *PayloadFields Structure to identify message formatting
   */
  template<etl::message_id_t MSG_ID, typename PayloadType, const size_t PayloadSize, const pb_msgdesc_t *PayloadFields>
  class MessageExt : public etl::message<MSG_ID>
  {
  public:
    PayloadType payload;

    /**
     * @brief Resets the message payload to empty
     */
    void reset()
    {
      bytesWritten = 0;
      ioBuffer.fill( 0 );
    }

    /**
     * @brief Encode the current state of the message to the internal data buffer
     * @return bool   True if encoding was successful, false if not
     */
    bool encode()
    {
      return this->encode( payload );
    }

    /**
     * @brief Encode the given message to the internal data buffer
     *
     * @param message   Message being encoded
     * @return bool     True if encoding was successful, false if not
     */
    bool encode( const PayloadType &message )
    {
      pb_ostream_t stream = pb_ostream_from_buffer( ioBuffer.data(), ioBuffer.size() );
      bool         result = pb_encode( &stream, PayloadFields, &message );
      bytesWritten        = stream.bytes_written;
      return result;
    }

    /**
     * @brief Decode a buffer of data into the internal message
     *
     * @param src     Source buffer of raw data to parse
     * @param length  Size of the bytes contained in the source buffer
     * @return bool   True if decoding was successful, false if not
     */
    bool decode( const void *const src, const size_t length )
    {
      return this->decode( payload, src, length );
    }

    /**
     * @brief Decodes a buffer of data under the assumption it contains the message type
     *
     * @param dst     Where to write the decoded data into
     * @param src     Source buffer of raw data to parse
     * @param length  Size of the bytes contained in the source buffer
     * @return bool   True if decoding was successful, false if not
     */
    bool decode( PayloadType &dst, const void *const src, const size_t length )
    {
      /*-----------------------------------------------------------------------
      Input Protection
      -----------------------------------------------------------------------*/
      if ( !src || ( length < PayloadSize ) )
      {
        RT_DBG_ASSERT( false );
        return false;
      }

      /*-----------------------------------------------------------------------
      Use the internal buffer to decode the stream
      -----------------------------------------------------------------------*/
      pb_istream_t stream = pb_istream_from_buffer( reinterpret_cast<const pb_byte_t *>( src ), PayloadSize );
      return pb_decode( &stream, PayloadFields, &dst );
    }

    /**
     * @brief Sends the encoded message data over a serial link
     *
     * @param serial     Serial port to write to
     * @return Chimera::Status_t
     */
    Chimera::Status_t send( Chimera::Serial::Driver_rPtr serial, const bool block = false )
    {
      using namespace Chimera::Event;
      using namespace Chimera::Thread;

      /*-----------------------------------------------------------------------
      Input Protection
      -----------------------------------------------------------------------*/
      RT_DBG_ASSERT( serial );
      if ( ( bytesWritten == 0 ) || ( bytesWritten > PayloadSize ) )
      {
        return Chimera::Status::FAIL;
      }

      /*-----------------------------------------------------------------------
      Ship the data through the port
      -----------------------------------------------------------------------*/
      auto result = serial->write( ioBuffer.data(), bytesWritten );
      if( block )
      {
        result |= serial->await( Trigger::TRIGGER_WRITE_COMPLETE, TIMEOUT_BLOCK );
      }

      return result;
    }

  protected:
    static_assert( sizeof( PayloadType ) <= PayloadSize );

    static constexpr size_t EncodeSize = COBS_ENCODE_DST_BUF_LEN_MAX( PayloadSize );
    static constexpr size_t DecodeSize = COBS_DECODE_DST_BUF_LEN_MAX( PayloadSize );
    static constexpr size_t IOBuffSize = std::max<size_t>( EncodeSize, DecodeSize );

    uint16_t                        bytesWritten;
    etl::array<uint8_t, IOBuffSize> ioBuffer;
  };


  /*---------------------------------------------------------------------------
  Message Class Declarations
  ---------------------------------------------------------------------------*/
  class Ping : public MessageExt<MSG_PING_CMD, PingMessage, PingMessage_size, PingMessage_fields>
  {
  };

}    // namespace Orbit::Serial::Message

#endif /* !ORBIT_SERIAL_MESSAGE_INTF_HPP */
