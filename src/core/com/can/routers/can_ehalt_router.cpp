/******************************************************************************
 *  File Name:
 *    can_ehalt_router.cpp
 *
 *  Description:
 *    Emergency Halt Message Router
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
  EmergencyHaltRouter::EmergencyHaltRouter() : message_router( ROUTER_ID_EMERGENCY_HALT )
  {
  }


  void EmergencyHaltRouter::on_receive( const Message::EmergencyHalt &msg )
  {
    // using namespace Orbit::Control;

    // if ( ( msg.payload.dst.nodeId == EnumValue( thisNode() ) ) || ( msg.payload.dst.nodeId == EnumValue( NodeId::NODE_ALL ) ) )
    // {
    //   LOG_WARN( "**** Emergency Halt Received ****\r\n" );
    //   FOCDriver.sendSystemEvent( EventId::EMERGENCY_HALT );
    // }
  }


  void EmergencyHaltRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
