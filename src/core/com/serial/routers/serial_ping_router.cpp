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
#include <src/core/hw/orbit_usart.hpp>

namespace Orbit::Serial::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  PingRouter::PingRouter() : message_router( Message::MSG_PING_CMD )
  {
  }


  void PingRouter::on_receive( const Message::Ping &msg )
  {
    /*-------------------------------------------------------------------------
    Populate the core message type
    -------------------------------------------------------------------------*/
    AckNackMessage payload;
    memset( &payload, 0, sizeof( payload ) );
    payload.header      = msg.payload.header;
    payload.acknowledge = true;

    /*-------------------------------------------------------------------------
    Ship the response on the wire
    -------------------------------------------------------------------------*/
    Message::AckNack reply;
    reply.reset();
    reply.encode( payload );
    reply.send( Orbit::USART::SerialDriver );
  }


  void PingRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }

}  // namespace Orbit::Serial::Router
