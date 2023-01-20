/******************************************************************************
 *  File Name:
 *    serial_router.hpp
 *
 *  Description:
 *    Routers for various Serial RX protocol buffer messages
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_ROUTERS_HPP
#define ORBIT_SERIAL_ROUTERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/serial/serial_async_message.hpp>
#include <etl/message_router.h>

namespace Orbit::Serial::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class PingRouter : public etl::message_router<PingRouter, Message::Ping>
  {
  public:
    PingRouter();
    void on_receive( const Message::Ping &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

}  // namespace Orbit::Serial::Router

#endif  /* !ORBIT_SERIAL_ROUTERS_HPP */
