/******************************************************************************
 *  File Name:
 *    can_message.cpp
 *
 *  Description:
 *    Implementation details for CAN messages
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/can_message.hpp>
#include <src/core/hw/orbit_can.hpp>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  System Tick Update
  ---------------------------------------------------------------------------*/
  void SystemTick::update()
  {
    payload.tick = static_cast<uint32_t>( Chimera::millis() );
    this->send( CANDriver );
  }
}    // namespace Orbit::CAN::Message
