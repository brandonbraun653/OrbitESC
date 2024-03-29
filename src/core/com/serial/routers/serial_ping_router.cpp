/******************************************************************************
 *  File Name:
 *    serial_ping_router.cpp
 *
 *  Description:
 *    Implementation of the Ping Router
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/serial/serial_router.hpp>
#include <src/core/com/serial/serial_server.hpp>

namespace Orbit::Serial::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  PingRouter::PingRouter() : message_router( ROUTER_ID_PING )
  {
  }


  void PingRouter::on_receive( const Message::Ping &msg )
  {
    sendAckNack( true, msg.raw.header );
  }


  void PingRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::Serial::Router
