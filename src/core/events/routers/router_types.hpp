/******************************************************************************
 *  File Name:
 *    router_types.hpp
 *
 *  Description:
 *    Various system message routers
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SYSTEM_EVENT_ROUTERS_HPP
#define ORBIT_ESC_SYSTEM_EVENT_ROUTERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/message_router.h>
#include <src/core/events/event_types.hpp>

namespace Orbit::Event
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum RouterId : etl::message_router_id_t
  {
    ROUTER_CORE_CONTROL = 0,
    ROUTER_CORE_DATA,
    ROUTER_CORE_COUNT
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class CoreControlRouter : public etl::message_router<CoreControlRouter, SystemReset, StreamPhaseCurrents>
  {
  public:
    CoreControlRouter();
    void on_receive( const SystemReset &msg );
    void on_receive( const StreamPhaseCurrents &msg );
    void on_receive_unknown(const etl::imessage& msg);
  };
}  // namespace Orbit::Event::Routers

#endif  /* !ORBIT_ESC_SYSTEM_EVENT_ROUTERS_HPP */
