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
#include "pb_decode.h"
#include "pb_encode.h"
#include <Chimera/common>
#include <Chimera/serial>
#include <array>
#include <etl/array.h>
#include <etl/message.h>
#include <src/core/com/serial/serial_interface.pb.h>

namespace Orbit::Serial::Message
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using UniqueId_t = uint16_t;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t _msg_size_array[] = { AckNackMessage_size, PingMessage_size, ConsoleMessage_size, SystemTick_size, SystemInfoMessage_size };
  static constexpr size_t MAX_RAW_MSG_SIZE  = *std::max_element( std::begin( _msg_size_array ), std::end( _msg_size_array ) );
  static constexpr size_t MAX_COBS_MSG_SIZE = COBS_ENCODE_DST_BUF_LEN_MAX( MAX_RAW_MSG_SIZE );
  static constexpr size_t MIN_RAW_MSG_SIZE  = sizeof( Header );
  static constexpr size_t MIN_COBS_MSG_SIZE = COBS_ENCODE_DST_BUF_LEN_MAX( MIN_RAW_MSG_SIZE );

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Alias the definitions from the protocol buffer interface spec
   */
  enum Id : etl::message_id_t
  {
    MSG_ACK_NACK = SerialMsgId_MSG_ACK_NACK,
    MSG_PING_CMD = SerialMsgId_MSG_PING_CMD,
    MSG_TERMINAL = SerialMsgId_MSG_TERMINAL,
    MSG_SYS_TICK = SerialMsgId_MSG_SYS_TICK,
    MSG_SYS_INFO = SerialMsgId_MSG_SYS_INFO,

    MSG_ID_COUNT
  };
  static_assert( MSG_ID_COUNT == ARRAY_COUNT( _msg_size_array ) );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the async message module
   */
  void moduleInit();

  /**
   * @brief Atomically retrieve the next unique message identifier
   * @return UniqueId_t
   */
  UniqueId_t getNextUUID();

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
    static constexpr etl::message_id_t MessageId = MSG_ID;
    PayloadType payload;

    /**
     * @brief Resets the message payload to empty
     */
    void reset()
    {
      bytes_written = 0;
      io_buffer.fill( 0 );
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
      /*-----------------------------------------------------------------------
      Reset the working memory
      -----------------------------------------------------------------------*/
      FrameBufferType pb_buffer;
      pb_buffer.fill( 0 );
      io_buffer.fill( 0 );
      bytes_written = 0;

      /*-----------------------------------------------------------------------
      Encode the data type using nanopb
      -----------------------------------------------------------------------*/
      pb_ostream_t stream    = pb_ostream_from_buffer( pb_buffer.data(), pb_buffer.size() );
      bool         pbSuccess = pb_encode( &stream, PayloadFields, &message );

      /*-----------------------------------------------------------------------
      Frame the transaction with COBS encoding. Add +1 to the total encoded
      size to account for the null byte delimiter that gets transmitted.
      -----------------------------------------------------------------------*/
      cobs_encode_result cobsResult = cobs_encode( io_buffer.data(), io_buffer.size(), pb_buffer.data(), stream.bytes_written );
      bytes_written                 = cobsResult.out_len + 1u;

      RT_DBG_ASSERT( bytes_written <= io_buffer.size() );
      return ( pbSuccess && ( cobsResult.status == COBS_ENCODE_OK ) );
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
     * @brief Decodes a buffer of data
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
      if ( !src || ( length > IOBuffSize ) )
      {
        RT_DBG_ASSERT( false );
        return false;
      }

      /*-----------------------------------------------------------------------
      Unpack the COBS encoded frame
      -----------------------------------------------------------------------*/
      cobs_decode_result cobsResult = cobs_decode( io_buffer.data(), io_buffer.size(), src, length );
      if( cobsResult.status != COBS_DECODE_OK )
      {
        return false;
      }

      /*-----------------------------------------------------------------------
      Decode the nanopb data
      -----------------------------------------------------------------------*/
      pb_istream_t stream = pb_istream_from_buffer( reinterpret_cast<const pb_byte_t *>( io_buffer.data() ), cobsResult.out_len );
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
      if ( ( bytes_written == 0 ) || ( bytes_written > PayloadSize ) )
      {
        return Chimera::Status::FAIL;
      }

      /*-----------------------------------------------------------------------
      Ship the data through the port
      -----------------------------------------------------------------------*/
      auto timeout = block ? TIMEOUT_BLOCK : TIMEOUT_DONT_WAIT;

      size_t offset = 0;
      size_t remaining = bytes_written;

      while( offset < bytes_written )
      {
        size_t act_written = serial->write( io_buffer.data() + offset, remaining, timeout );

        offset += act_written;
        remaining -= act_written;

        RT_DBG_ASSERT( remaining <= bytes_written );
      }

      return ( offset == bytes_written ) ? Chimera::Status::OK : Chimera::Status::FAIL;
    }

  protected:
    static_assert( sizeof( PayloadType ) <= PayloadSize );

    static constexpr size_t EncodeSize = COBS_ENCODE_DST_BUF_LEN_MAX( PayloadSize );
    static constexpr size_t DecodeSize = COBS_DECODE_DST_BUF_LEN_MAX( PayloadSize );
    static constexpr size_t IOBuffSize = std::max<size_t>( EncodeSize, DecodeSize ) + 1u;

    using FrameBufferType = etl::array<uint8_t, IOBuffSize>;

    uint16_t        bytes_written;
    FrameBufferType io_buffer;
  };


  /*---------------------------------------------------------------------------
  Message Class Declarations
  ---------------------------------------------------------------------------*/
  class AckNack : public MessageExt<MSG_ACK_NACK, AckNackMessage, AckNackMessage_size, AckNackMessage_fields>
  {
  };

  class Ping : public MessageExt<MSG_PING_CMD, PingMessage, PingMessage_size, PingMessage_fields>
  {
  };

  class Console : public MessageExt<MSG_TERMINAL, ConsoleMessage, ConsoleMessage_size, ConsoleMessage_fields>
  {
  };

  class SysTick : public MessageExt<MSG_SYS_TICK, SystemTick, SystemTick_size, SystemTick_fields>
  {
  };

  class SysInfo : public MessageExt<MSG_SYS_INFO, SystemInfoMessage, SystemInfoMessage_size, SystemInfoMessage_fields>
  {
  };

}    // namespace Orbit::Serial::Message

#endif /* !ORBIT_SERIAL_MESSAGE_INTF_HPP */
