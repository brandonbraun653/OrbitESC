/******************************************************************************
 *  File Name:
 *    can_speed_router.cpp
 *
 *  Description:
 *    Ping Message Router
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/can/can_message.hpp>
#include <src/core/com/can/can_router.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/runtime/can_runtime.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/foc_driver.hpp>

namespace Orbit::CAN::Router
{
  /*---------------------------------------------------------------------------
  SetMotorSpeed
  ---------------------------------------------------------------------------*/
  SetMotorSpeedRouter::SetMotorSpeedRouter() : message_router( Message::MSG_SET_MOTOR_SPEED )
  {
  }


  void SetMotorSpeedRouter::on_receive( const Message::SetMotorSpeed &msg )
  {
    //Orbit::Control::FOCDriver.setSpeedRef( RPM_TO_RAD( msg.payload.speed ) );
  }


  void SetMotorSpeedRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
