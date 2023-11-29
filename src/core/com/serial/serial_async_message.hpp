/******************************************************************************
 *  File Name:
 *    serial_message_intf.hpp
 *
 *  Description:
 *    Common serial message interface
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
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
#include <etl/vector.h>
#include <src/core/com/proto/serial_interface.pb.h>

namespace Orbit::Serial::Message
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using UniqueId_t = uint16_t;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  namespace Internal
  {
    static constexpr size_t _msg_size_array[] = {
      /* clang-format off */
      AckNackMessage_size,
      PingMessage_size,
      ConsoleMessage_size,
      SystemTick_size,
      SystemInfoMessage_size,
      ParamIOMessage_size,
      SystemControlMessage_size,
      SwitchModeMessage_size,
      SystemDataMessage_size,
      /* clang-format on */
    };
  }
  static constexpr size_t MAX_RAW_MSG_SIZE =
      *std::max_element( std::begin( Internal::_msg_size_array ), std::end( Internal::_msg_size_array ) );
  static constexpr size_t MIN_RAW_MSG_SIZE  = sizeof( Header );
  static constexpr size_t MAX_COBS_MSG_SIZE = COBS_ENCODE_DST_BUF_LEN_MAX( MAX_RAW_MSG_SIZE ) + 1u;
  static constexpr size_t MIN_COBS_MSG_SIZE = COBS_ENCODE_DST_BUF_LEN_MAX( MIN_RAW_MSG_SIZE ) + 1u;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Alias the definitions from the protocol buffer interface spec
   */
  enum _Id : etl::message_id_t
  {
    MSG_ACK_NACK    = MsgId_MSG_ACK_NACK,
    MSG_PING_CMD    = MsgId_MSG_PING_CMD,
    MSG_TERMINAL    = MsgId_MSG_TERMINAL,
    MSG_SYS_TICK    = MsgId_MSG_SYS_TICK,
    MSG_SYS_INFO    = MsgId_MSG_SYS_INFO,
    MSG_PARAM_IO    = MsgId_MSG_PARAM_IO,
    MSG_SYS_CTRL    = MsgId_MSG_SYS_CTRL,
    MSG_SWITCH_MODE = MsgId_MSG_SWITCH_MODE,
    MSG_SYS_DATA    = MsgId_MSG_SYS_DATA,

    MSG_ID_COUNT
  };
  static_assert( MSG_ID_COUNT == ARRAY_COUNT( Internal::_msg_size_array ) );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Datastructure for decoupling the encoding/decoding process from message type declaration.
   */
  struct EncodableMessage
  {
    pb_byte_t *const          IOBuffer;        /**< Persistent IO buffer for COBS encoding */
    const size_t              IOBufferSize;    /**< Size of the IO buffer */
    const pb_msgdesc_t *const PBPayloadFields; /**< Descriptor of PB encoding type */
    const size_t              PBPayloadSize;   /**< Size of the PB data structure */
    void *const               PBPayloadData;   /**< Pointer to the PB data structure */
    size_t                    EncodedSize;     /**< Size of the encoded message */
  };

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

  /**
   * @brief Decodes a buffer of data
   *
   * @param msg     Message descriptor to use for decoding
   * @param src     Source buffer of raw data to decode
   * @param length  Size of the bytes contained in the source buffer
   * @return bool   True if decoding was successful, false if not
   */
  bool decode( EncodableMessage *const msg, const void *const src, const size_t length );

  /**
   * @brief Encodes an external buffer of data
   *
   * @param msg     Message descriptor to use for encoding
   * @param src     Source buffer of raw data to encode
   * @param length  Size of the bytes contained in the source buffer
   * @return bool     True if encoding was successful, false if not
   */
  bool encode( EncodableMessage *const msg, const void *const src, const size_t length );

  /**
   * @brief Encodes the internal message data
   *
   * @param msg   Message descriptor to use for encoding
   * @return bool True if encoding was successful, false if not
   */
  bool encode( EncodableMessage *const msg );

  /**
   * @brief Sends the encoded message data over a serial link
   *
   * @param msg        Message descriptor to send
   * @param serial     Serial port to write to
   * @return Chimera::Status_t
   */
  Chimera::Status_t send( const EncodableMessage *const msg, Chimera::Serial::Driver_rPtr serial, const bool block = false );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Template class to specialize a declaration of a serial message.
   *
   * This will define all parameters required to encode/decode a message at runtime as
   * well as provide persistent storage for the message payload. This is required for
   * various IO drivers to work properly.
   *
   * @tparam MSG_ID         Field from Orbit::Serial::Message::Id
   * @tparam PayloadType    Structure type generated from proto file
   * @tparam PayloadSize    Maximum size of the encoded message
   * @tparam *PayloadFields Structure to identify message formatting
   */
  template<etl::message_id_t MSG_ID, typename PayloadType, const size_t PayloadSize, const pb_msgdesc_t *const PayloadFields>
  class _CustomMsg : public etl::message<MSG_ID>
  {
  public:
    static constexpr size_t            EncodeSize = COBS_ENCODE_DST_BUF_LEN_MAX( PayloadSize );
    static constexpr size_t            DecodeSize = COBS_DECODE_DST_BUF_LEN_MAX( PayloadSize );
    static constexpr size_t            IOBuffSize = std::max<size_t>( EncodeSize, DecodeSize ) + 1u;
    static constexpr etl::message_id_t MessageId  = MSG_ID;

    PayloadType  payload; /**< Raw data structure of the message */
    EncodableMessage state;   /**< Encoder state data */

    _CustomMsg() : payload{ 0 }, state{ io_buffer, IOBuffSize, PayloadFields, PayloadSize, &payload, 0 }
    {
    }

  private:
    pb_byte_t io_buffer[ IOBuffSize ]; /**< Buffer for COBS encoding */
  };


  /*---------------------------------------------------------------------------
  Message Class Declarations
  ---------------------------------------------------------------------------*/
  using AckNack    = _CustomMsg<MSG_ACK_NACK, AckNackMessage, AckNackMessage_size, AckNackMessage_fields>;
  using Ping       = _CustomMsg<MSG_PING_CMD, PingMessage, PingMessage_size, PingMessage_fields>;
  using Console    = _CustomMsg<MSG_TERMINAL, ConsoleMessage, ConsoleMessage_size, ConsoleMessage_fields>;
  using SysTick    = _CustomMsg<MSG_SYS_TICK, SystemTick, SystemTick_size, SystemTick_fields>;
  using SysInfo    = _CustomMsg<MSG_SYS_INFO, SystemInfoMessage, SystemInfoMessage_size, SystemInfoMessage_fields>;
  using ParamIO    = _CustomMsg<MSG_PARAM_IO, ParamIOMessage, ParamIOMessage_size, ParamIOMessage_fields>;
  using SysCtrl    = _CustomMsg<MSG_SYS_CTRL, SystemControlMessage, SystemControlMessage_size, SystemControlMessage_fields>;
  using SwitchMode = _CustomMsg<MSG_SWITCH_MODE, SwitchModeMessage, SwitchModeMessage_size, SwitchModeMessage_fields>;
  using SysData    = _CustomMsg<MSG_SYS_DATA, SystemDataMessage, SystemDataMessage_size, SystemDataMessage_fields>;

}    // namespace Orbit::Serial::Message

#endif /* !ORBIT_SERIAL_MESSAGE_INTF_HPP */
