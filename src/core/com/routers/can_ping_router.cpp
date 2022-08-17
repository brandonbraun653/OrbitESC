/******************************************************************************
 *  File Name:
 *    can_ping_router.cpp
 *
 *  Description:
 *    Ping Message Router
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/core/com/can_message.hpp>
#include <src/core/com/can_router.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/runtime/can_runtime.hpp>

namespace Orbit::CAN::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  PingRouter::PingRouter() : message_router( Message::MSG_PING )
  {
  }


  void PingRouter::on_receive( const Message::Ping &msg )
  {
    Chimera::CAN::BasicFrame tx_frame;
    Message::Ping pong;

    /* Reverse the source/destinations */
    pong.payload.dst.nodeId = msg.payload.src.nodeId;
    pong.payload.src.nodeId = EnumValue( thisNode() );

    /* Re-pack and ship it back */
    pong.pack( tx_frame );
    CANDriver->send( tx_frame );

    LOG_INFO( "ACK ping from %s\r\n", getNodeName( static_cast<NodeId>( msg.payload.src.nodeId ) ).data() );
  }


  void PingRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::CAN
