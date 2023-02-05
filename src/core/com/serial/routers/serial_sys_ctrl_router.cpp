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
    switch ( msg.payload.header.subId )
    {
      case Message::SUB_MSG_SYS_CTRL_RESET:
        sendAckNack( true, msg.payload.header );
        Chimera::delayMilliseconds( 50 );
        Orbit::System::doSafeShutdown();
        break;

      default:
        sendAckNack( false, msg.payload.header );
        break;
    }
  }

  void SysCtrlRouter::on_receive_unknown( const etl::imessage &msg )
  {
    // Do nothing
  }
}    // namespace Orbit::Serial::Router
