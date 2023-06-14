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
#include <src/core/hw/orbit_can.hpp>
#include <src/core/com/can/can_server.hpp>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern Server MessageServer;  /**< Driver for the board's CAN message server */


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the CAN bus runtime driver for the project
   */
  void initRuntime();

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

}  // namespace Orbit::CAN

#endif  /* !ORBIT_CAN_RUNTIME_HPP */
