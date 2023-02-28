/******************************************************************************
 *  File Name:
 *    serial_switch_mode_router.cpp
 *
 *  Description:
 *    Implementation of the Switch Mode Router
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
  SwitchModeRouter::SwitchModeRouter() : message_router( Message::MSG_SWITCH_MODE )
  {
  }

  void SwitchModeRouter::on_receive( const Message::SwitchMode &msg )
  {
    sendAckNack( true, msg.payload.header );
    Chimera::delayMilliseconds( 50 );
    System::setMode( static_cast<System::Mode>( msg.payload.mode ) );
  }


  void SwitchModeRouter::on_receive_unknown( const etl::imessage &msg )
  {
    // Do nothing
  }

}  // namespace Orbit::Serial::Router
