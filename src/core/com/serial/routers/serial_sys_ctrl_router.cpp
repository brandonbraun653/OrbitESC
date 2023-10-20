/******************************************************************************
 *  File Name:
 *    serial_sys_ctrl_router.cpp
 *
 *  Description:
 *    System control message router
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/serial/serial_router.hpp>
#include <src/core/com/serial/serial_server.hpp>
#include <src/core/system.hpp>
#include <src/control/current_control.hpp>
#include <src/core/hw/orbit_motor.hpp>

namespace Orbit::Serial::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  SysCtrlRouter::SysCtrlRouter() : message_router( Message::MSG_SYS_CTRL )
  {
  }

  void SysCtrlRouter::on_receive( const Message::SysCtrl &msg )
  {
    bool should_ack = false;

    switch ( msg.payload.header.subId )
    {
      /*-----------------------------------------------------------------------
      Do a system reset. Send the ACK first so the host knows it was received.
      -----------------------------------------------------------------------*/
      case SubId_SUB_MSG_SYS_CTRL_RESET:
        sendAckNack( true, msg.payload.header );
        Chimera::delayMilliseconds( 50 );
        Orbit::System::setMode( Orbit::System::getMode() );
        return;

      /*-----------------------------------------------------------------------
      Handle motor control messages
      -----------------------------------------------------------------------*/
      case SubId_SUB_MSG_SYS_CTRL_MOTOR:
        if ( msg.payload.has_motorCmd )
        {
          should_ack = true;

          switch ( msg.payload.motorCmd )
          {
            case MotorCtrlCmd_ENABLE_OUTPUT_STAGE:
              Motor::enableDriveOutput();
              break;

            case MotorCtrlCmd_DISABLE_OUTPUT_STAGE:
              Motor::disableDriveOutput();
              break;

            case MotorCtrlCmd_EMERGENCY_STOP:
              Motor::emergencyStop();
              break;

            default:
              should_ack = false;
              break;
          }
        }
        break;

      /*-----------------------------------------------------------------------
      Enable/disable the inner loop manual control
      -----------------------------------------------------------------------*/
      case SubId_SUB_MSG_SYS_CTRL_MANUAL_INNER_LOOP:
        if ( Control::Field::getControlMode() == Control::Field::Mode::OPEN_LOOP )
        {
          should_ack = Control::Field::setControlMode( Control::Field::Mode::DISABLED );
        }
        else
        {
          should_ack = Control::Field::setControlMode( Control::Field::Mode::OPEN_LOOP );
        }
        break;

      /*-----------------------------------------------------------------------
      Assign new references for the inner loop
      -----------------------------------------------------------------------*/
      case SubId_SUB_MSG_SYS_CTRL_MANUAL_INNER_LOOP_REF:
        if ( msg.payload.has_data && ( msg.payload.data.size == sizeof( SystemControlMessage_ManualICtrlSetPoint ) ) )
        {
          should_ack       = true;
          const auto pData = reinterpret_cast<const SystemControlMessage_ManualICtrlSetPoint *>( msg.payload.data.bytes );
          Control::Field::setInnerLoopReferences( pData->iq_ref, pData->id_ref, pData->rotor_theta_rad );
        }
        break;

      /*-----------------------------------------------------------------------
      Unhandled message. Default to NACK.
      -----------------------------------------------------------------------*/
      default:
        should_ack = false;
        break;
    }

    /*-------------------------------------------------------------------------
    Send the results back to the sender
    -------------------------------------------------------------------------*/
    sendAckNack( should_ack, msg.payload.header );
  }

  void SysCtrlRouter::on_receive_unknown( const etl::imessage &msg )
  {
    // Do nothing
  }
}    // namespace Orbit::Serial::Router
