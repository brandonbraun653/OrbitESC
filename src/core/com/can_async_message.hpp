/******************************************************************************
 *  File Name:
 *    can_async_message.hpp
 *
 *  Description:
 *    Asynchronous message definitions
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_ASYNC_MESSAGE_HPP
#define ORBIT_CAN_ASYNC_MESSAGE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/can_message.hpp>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  Asynchronous Messages: See can_message.hpp for descriptions
  ---------------------------------------------------------------------------*/
  class Ping : public Attributes<Ping, MSG_PING>
  {
  public:
    __packed_struct Payload
    {
      Header dst; /**< Destination node */
      Header src; /**< Source node */
    }
    payload;
  };


  class SetSystemMode : public Attributes<SetSystemMode, MSG_SET_SYSTEM_MODE>
  {
  public:
    __packed_struct Payload
    {
      Header              dst;  /**< Destination node */
      etl::fsm_state_id_t mode; /**< New mode to switch into Orbit::Control::ModeId */
    }
    payload;
  };


  class SetMotorSpeed : public Attributes<SetMotorSpeed, MSG_SET_MOTOR_SPEED>
  {
  public:
    __packed_struct Payload
    {
      Header   dst;   /**< Destination node */
      uint16_t speed; /**< Motor speed in RPM */
    }
    payload;
  };


  class SetConfigData : public Attributes<SetConfigData, MSG_SET_CONFIG_DATA>
  {
  public:
    __packed_struct Payload
    {
      Header  dst;       /**< Destination node */
      uint8_t id;        /**< Configuration ID */
      uint8_t data[ 6 ]; /**< Configuration data */
    }
    payload;
  };


  class GetConfigData : public Attributes<GetConfigData, MSG_GET_CONFIG_DATA>
  {
  public:
    __packed_struct Payload
    {
      Header  dst; /**< Destination node */
      uint8_t id;  /**< Configuration ID */
    }
    payload;
  };


  class RspConfigData : public Attributes<RspConfigData, MSG_RSP_CONFIG_DATA>
  {
  public:
    __packed_struct Payload
    {
      Header  src;       /**< Destination node */
      uint8_t id;        /**< Configuration ID */
      uint8_t data[ 6 ]; /**< Configuration data */
    }
    payload;
  };


  class EmergencyHalt : public Attributes<EmergencyHalt, MSG_EMERGENCY_HALT>
  {
  public:
    __packed_struct Payload
    {
      Header dst; /**< Destination node */
    }
    payload;
  };


  class SystemReset : public Attributes<SystemReset, MSG_SYSTEM_RESET>
  {
  public:
    __packed_struct Payload
    {
      Header dst; /**< Destination node */
    }
    payload;
  };


  class StreamBookend : public Attributes<StreamBookend, MSG_STREAM_BOOKEND>
  {
  public:
    __packed_struct Payload
    {
      Header   src;   /**< Source node */
      uint8_t  type;  /**< Type of stream */
      uint16_t bytes; /**< Number of bytes in the stream */
    }
    payload;
  };


  class StreamData : public Attributes<StreamData, MSG_STREAM_DATA>
  {
  public:
    __packed_struct Payload
    {
      Header  src;       /**< Source node */
      uint8_t type;      /**< Type of stream */
      uint8_t frame;     /**< Frame number of the stream */
      uint8_t data[ 5 ]; /**< Variable data amount  */
    }
    payload;
  };

}    // namespace Orbit::CAN::Message

#endif /* !ORBIT_CAN_ASYNC_MESSAGE_HPP */
