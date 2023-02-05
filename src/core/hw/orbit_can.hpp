/******************************************************************************
 *  File Name:
 *    orbit_can.hpp
 *
 *  Description:
 *    Orbit ESC CAN Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_CAN_HPP
#define ORBIT_ESC_CAN_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>
#include <cstdint>


namespace Orbit::CAN
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
    NODE_PC,
    NODE_ALL,

    NUM_SUPPORTED_NODES,
    INVALID
  };

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern Chimera::CAN::Driver_rPtr CANDriver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the CAN driver subsystem
   */
  void powerUp();

  /**
   * @brief Get a node id's name as a string
   *
   * @param id  Which ID to look up
   * @return const std::string_view&
   */
  const std::string_view& getNodeName( const NodeId id );

}    // namespace Orbit::CAN

#endif /* !ORBIT_ESC_CAN_HPP */
