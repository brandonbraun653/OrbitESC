/******************************************************************************
 *  File Name:
 *    serial_router.hpp
 *
 *  Description:
 *    Routers for various Serial RX protocol buffer messages
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_ROUTERS_HPP
#define ORBIT_SERIAL_ROUTERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/serial/serial_async_message.hpp>
#include <etl/message_router.h>
#include <etl/queue_spsc_atomic.h>

namespace Orbit::Serial::Router
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using ParamIOQueue = etl::queue_spsc_atomic<Message::ParamIO, 3, etl::memory_model::MEMORY_MODEL_SMALL>;

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  /**
   * @brief SPSC queue for Parameter IO messages received
   *
   * Producer: ParamIORouter
   * Consumer: DIOThread
   */
  extern ParamIOQueue ParamIOEventQueue;

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

  class ParamIORouter : public etl::message_router<ParamIORouter, Message::ParamIO>
  {
  public:
    ParamIORouter();
    void on_receive( const Message::ParamIO &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

  class SysCtrlRouter : public etl::message_router<SysCtrlRouter, Message::SysCtrl>
  {
  public:
    SysCtrlRouter();
    void on_receive( const Message::SysCtrl &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

}  // namespace Orbit::Serial::Router

#endif  /* !ORBIT_SERIAL_ROUTERS_HPP */
