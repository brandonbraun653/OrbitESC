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
#include <cstdint>
#include <etl/message.h>
#include <src/core/com/can_com_intf.hpp>
#include <Chimera/can>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Unique ID for a node in the ESC system
   */
  enum class NodeId : uint8_t
  {
    NODE_0,
    NODE_1,
    NODE_2,
    NODE_3,
    NODE_4,
    NODE_5,
    NODE_PC,

    NUM_SUPPORTED_NODES
  };

  enum Id : etl::message_id_t
  {
    /*-----------------------------------------------------------------------
    Periodic Messages (Must go first)
    -----------------------------------------------------------------------*/
    MSG_PERIODIC_START = 0,
    MSG_SYSTEM_TICK    = MSG_PERIODIC_START,

    // Add more periodic messages here. Update "END" enum.
    MSG_PERIODIC_END,

    /*-----------------------------------------------------------------------
    Async Messages
    -----------------------------------------------------------------------*/
    MSG_ASYNC_START = MSG_PERIODIC_END,
    MSG_PING        = MSG_ASYNC_START,

    // Add more async messages here. Update "END" enum.
    MSG_ASYNC_END,

    /*-----------------------------------------------------------------------
    Summary of enumerated values
    -----------------------------------------------------------------------*/
    MSG_INVALID,
    NUM_SUPPORTED_MSG = MSG_ASYNC_END,
    NUM_PERIODIC_MSG  = MSG_PERIODIC_END - MSG_PERIODIC_START,
    NUM_ASYNC_MSG     = MSG_ASYNC_END - MSG_ASYNC_START
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Describes the ESC that generated a message
   */
  struct SystemID
  {
    NodeId  nodeId : 3; /**< Unique node */
    uint8_t _pad : 5;   /**< Reserved for future use */
  };
  static_assert( sizeof( SystemID ) == sizeof( uint8_t ) );


  template<class Base, uint32_t ARB_ID, uint32_t PERIOD_MS = 0>
  class CanAttr
  {
  public:
    constexpr size_t messageSize() const
    {
      static_assert( sizeof( Base::Payload ) <= Chimera::CAN::MAX_PAYLOAD_LENGTH );
      return sizeof( Base::Payload );
    }

    constexpr uint32_t arbitrationId() const
    {
      return ARB_ID;
    }

    constexpr uint32_t period() const
    {
      return PERIOD_MS;
    }

    constexpr bool isPeriodic() const
    {
      return ( PERIOD_MS != 0 );
    }

    // pack/unpack functions. Pay the penalty of types for now.

  };


  class SystemTick : public etl::message<MSG_SYSTEM_TICK, CanAttr<SystemTick, 0x10>>
  {
  public:
    struct Payload
    {
      SystemID src;  /**< System information */
      uint32_t tick; /**< Current tick of system */
    } __attribute__( ( packed ) ); // Clean this up!
  };
}  // namespace Orbit::CAN

#endif  /* !ORBIT_CAN_MESSAGES_HPP */
