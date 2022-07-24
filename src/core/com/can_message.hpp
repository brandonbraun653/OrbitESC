/******************************************************************************
 *  File Name:
 *    can_message.hpp
 *
 *  Description:
 *    CAN bus message Definitions
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_MESSAGES_HPP
#define ORBIT_CAN_MESSAGES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/can_message_intf.hpp>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum Id : etl::message_id_t
  {
    MSG_SYSTEM_TICK = 0x15,
    MSG_PING        = 0x10,
  };


  /*---------------------------------------------------------------------------
  Message Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Ping message to test the connection
   */
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


  /**
   * @brief System time of the ESC node
   */
  class SystemTick : public Attributes<SystemTick, MSG_SYSTEM_TICK, 1000>
  {
  public:
    __packed_struct Payload
    {
      Header   hdr;  /**< Message Header */
      uint32_t tick; /**< Current tick of system in milliseconds */
    }
    payload;

    void update() final override;
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Returns the default frame for a CAN message
   *
   * @return Chimera::CAN::BasicFrame
   */
  static inline Chimera::CAN::BasicFrame defaultFrame()
  {
    Chimera::CAN::BasicFrame msg;
    msg.clear();
    msg.idMode    = Chimera::CAN::IdType::STANDARD;
    msg.frameType = Chimera::CAN::FrameType::DATA;

    return msg;
  }


  static constexpr size_t numPeriodicMessageTypes()
  {
    return calcPeriodicMessageTypes<Ping, SystemTick>();
  }

}    // namespace Orbit::CAN::Message

#endif /* !ORBIT_CAN_MESSAGES_HPP */
