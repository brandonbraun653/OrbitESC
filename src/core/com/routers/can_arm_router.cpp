/******************************************************************************
 *  File Name:
 *    can_arm_router.cpp
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
  Classes
  ---------------------------------------------------------------------------*/
  ArmDisarmMotorRouter::ArmDisarmMotorRouter() : message_router( Message::MSG_ARM_DISARM_MOTOR )
  {
  }


  void ArmDisarmMotorRouter::on_receive( const Message::ArmDisarmMotor &msg )
  {
    Chimera::CAN::BasicFrame tx_frame;
    // Send ACK
    // Set the new arm reference
  }


  void ArmDisarmMotorRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
