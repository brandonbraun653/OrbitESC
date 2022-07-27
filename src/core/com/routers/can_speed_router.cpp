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
#include <src/core/com/can_message.hpp>
#include <src/core/com/can_router.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/runtime/can_runtime.hpp>

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
    Chimera::CAN::BasicFrame tx_frame;
    // Send ACK
    // Set the new speed reference
  }


  void SetMotorSpeedRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }


  /*---------------------------------------------------------------------------
  GetMotorSpeed
  ---------------------------------------------------------------------------*/
  GetMotorSpeedRouter::GetMotorSpeedRouter() : message_router( Message::MSG_GET_MOTOR_SPEED )
  {
  }

  void GetMotorSpeedRouter::on_receive( const Message::GetMotorSpeed &msg )
  {
    Chimera::CAN::BasicFrame tx_frame;
    // Get the current speed reference and ship it back populated
  }

  void GetMotorSpeedRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
