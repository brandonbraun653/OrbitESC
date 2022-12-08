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
#include <src/core/com/can/can_message.hpp>
#include <src/core/com/can/can_router.hpp>
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
    using namespace Orbit::Control;

    int result = -1;
    auto desired_mode = static_cast<ModeId_t>( msg.payload.mode );

    switch ( desired_mode )
    {
      case ModeId::IDLE:
        result = FOCDriver.sendSystemEvent( EventId::DISARM );
        break;

      case ModeId::ARMED:
        result = FOCDriver.sendSystemEvent( EventId::ARM );
        break;

      case ModeId::ENGAGED:
        result = FOCDriver.sendSystemEvent( EventId::ENGAGE );
        break;

      case ModeId::FAULT:
        result = FOCDriver.sendSystemEvent( EventId::FAULT );
        break;

      default:
        // Do nothing
        break;
    };

    LOG_INFO( "Switch to mode %s %s\r\n", getModeString( desired_mode ).data(), result == 0 ? "success" : "failure" );
  }


  void SetSystemModeRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
