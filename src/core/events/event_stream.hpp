/******************************************************************************
 *  File Name:
 *    event_stream.hpp
 *
 *  Description:
 *    Event stream interface to support dynamic event propagation throughout
 *    the system.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_EVENT_STREAM_HPP
#define ORBIT_ESC_EVENT_STREAM_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/message_bus.h>
#include <etl/message.h>
#include <src/core/events/event_types.hpp>


namespace Orbit::Event
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  static constexpr size_t MAX_LISTENERS = 8;


  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/

  extern etl::message_bus<MAX_LISTENERS> gControlBus;  /**< System Control Events */
  extern etl::message_bus<MAX_LISTENERS> gDataBus;     /**< System Data Events */


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the event stream module
   * @return void
   */
  void initialize();
}    // namespace Orbit::Event

#endif /* !ORBIT_ESC_EVENT_STREAM_HPP */
