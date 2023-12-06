/******************************************************************************
 *  File Name:
 *    can_reset_router.cpp
 *
 *  Description:
 *    SystemReset Message Router
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/system>
#include <src/core/com/can/can_message.hpp>
#include <src/core/com/can/can_router.hpp>
#include <src/core/hw/orbit_can.hpp>

namespace Orbit::CAN::Router
{
  /*---------------------------------------------------------------------------
  SetMotorSpeed
  ---------------------------------------------------------------------------*/
  SystemResetRouter::SystemResetRouter() : message_router( ROUTER_ID_SYSTEM_RESET )
  {
  }


  void SystemResetRouter::on_receive( const Message::SystemReset &msg )
  {
    LOG_INFO( "System reset command received\r\n" );
    Chimera::System::softwareReset();
  }


  void SystemResetRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
