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
#include <src/control/current_control.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/events/event_stream.hpp>
#include <src/core/tasks.hpp>

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
    using namespace Orbit::Tasks;
    using namespace Chimera::Thread;

    bool should_ack = true;

    switch ( msg.raw.header.subId )
    {
      /*-----------------------------------------------------------------------
      Do a system reset. Send the ACK first so the host knows it was received.
      -----------------------------------------------------------------------*/
      case SystemControlSubId_RESET:
        sendAckNack( true, msg.raw.header );
        Orbit::Event::gControlBus.receive( Event::SystemReset() );
        return;

      case SystemControlSubId_DISABLE_STREAM_PHASE_CURRENTS:
      case SystemControlSubId_ENABLE_STREAM_PHASE_CURRENTS: {
        Event::StreamPhaseCurrents event;
        event.enable = ( msg.raw.header.subId == SystemControlSubId_ENABLE_STREAM_PHASE_CURRENTS );

        Orbit::Event::gControlBus.receive( event );
        break;
      }

      /*-----------------------------------------------------------------------
      Handle control system mode changes
      -----------------------------------------------------------------------*/
      case SystemControlSubId_ARM:
        should_ack = sendTaskMsg( getTaskId( TASK_CTL ), TASK_MSG_CTRL_ARM, TIMEOUT_BLOCK );
        break;

      case SystemControlSubId_DISARM:
        should_ack = sendTaskMsg( getTaskId( TASK_CTL ), TASK_MSG_CTRL_DISARM, TIMEOUT_BLOCK );
        break;

      case SystemControlSubId_ENGAGE:
        should_ack = sendTaskMsg( getTaskId( TASK_CTL ), TASK_MSG_CTRL_ENGAGE, TIMEOUT_BLOCK );
        break;

      case SystemControlSubId_DISENGAGE:
        should_ack = sendTaskMsg( getTaskId( TASK_CTL ), TASK_MSG_CTRL_DISENGAGE, TIMEOUT_BLOCK );
        break;

      case SystemControlSubId_FAULT:
        should_ack = sendTaskMsg( getTaskId( TASK_CTL ), TASK_MSG_CTRL_FAULT, TIMEOUT_BLOCK );
        break;

      case SystemControlSubId_EMERGENCY_STOP:
        should_ack = sendTaskMsg( getTaskId( TASK_CTL ), TASK_MSG_CTRL_EMERGENCY_HALT, TIMEOUT_BLOCK );
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
    sendAckNack( should_ack, msg.raw.header );
  }

  void SysCtrlRouter::on_receive_unknown( const etl::imessage &msg )
  {
    // Do nothing
  }
}    // namespace Orbit::Serial::Router
