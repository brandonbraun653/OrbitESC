/******************************************************************************
 *  File Name:
 *    event_stream.cpp
 *
 *  Description:
 *    System event stream implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/events/event_stream.hpp>
#include <src/core/events/routers/router_types.hpp>

namespace Orbit::Event
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  etl::message_bus<MAX_LISTENERS> gControlBus;
  etl::message_bus<MAX_LISTENERS> gDataBus;

  /*---------------------------------------------------------------------------
  Private Data
  ---------------------------------------------------------------------------*/
  static CoreControlRouter sCoreControlRouter;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize the system control bus router
    -------------------------------------------------------------------------*/
    gControlBus.clear();
    gControlBus.subscribe( sCoreControlRouter );

    /*-------------------------------------------------------------------------
    Initialize the system data bus router
    -------------------------------------------------------------------------*/
    gDataBus.clear();
  }
}    // namespace Orbit::Event
