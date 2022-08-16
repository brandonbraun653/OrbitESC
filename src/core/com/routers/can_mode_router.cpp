/******************************************************************************
 *  File Name:
 *    can_mode_router.cpp
 *
 *  Description:
 *    System Mode Message Router
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/core/com/can_message.hpp>
#include <src/core/com/can_router.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/runtime/can_runtime.hpp>
#include <src/control/foc_driver.hpp>

namespace Orbit::CAN::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  SetSystemModeRouter::SetSystemModeRouter() : message_router( Message::MSG_SET_SYSTEM_MODE )
  {
  }


  void SetSystemModeRouter::on_receive( const Message::SetSystemMode &msg )
  {
    // Orbit::Control::FOCDriver
    // Chimera::CAN::BasicFrame tx_frame;
    // Send ACK
    // Set the new arm reference
    LOG_INFO( "SetSystemModeRouter::on_receive()\r\n" );
  }


  void SetSystemModeRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
