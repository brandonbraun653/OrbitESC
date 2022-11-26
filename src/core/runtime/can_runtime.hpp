/******************************************************************************
 *  File Name:
 *    can_runtime.hpp
 *
 *  Description:
 *    CAN bus runtime operations
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_RUNTIME_HPP
#define ORBIT_CAN_RUNTIME_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>
#include <src/core/com/can/can_message.hpp>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Types
  ---------------------------------------------------------------------------*/
  using MessageHandler = void ( * )( const Chimera::CAN::BasicFrame & );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the CAN bus runtime driver for the project
   */
  void initRuntime();

  /**
   * @brief Assigns the handler used for a particular message type
   *
   * @param msg_enum  What message enumeration to register against
   * @param handler   Function pointer
   *
   * @return true     Registered successfully
   * @return false    Not registered
   */
  bool setHandler( uint8_t msg_enum, MessageHandler handler );

  /**
   * @brief Gets the node identifier for this CAN device
   *
   * @return NodeId
   */
  NodeId thisNode();

  /**
   * @brief Handles runtime IO of CAN bus messages. Must be called periodically.
   */
  void processCANBus();

  /**
   * @brief Transmits the system tick to the network for this node
   *
   */
  void periodicTXSystemTick();

}  // namespace Orbit::CAN

#endif  /* !ORBIT_CAN_RUNTIME_HPP */
