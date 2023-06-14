/******************************************************************************
 *  File Name:
 *    orbit_current_monitor.hpp
 *
 *  Description:
 *    Current Monitoring Solution
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CURRENT_MONITOR_HPP
#define ORBIT_CURRENT_MONITOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_analog_monitor.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class CurrentMonitor : public IAnalogMonitor
  {
  public:
    CurrentMonitor();
    ~CurrentMonitor();

    /*-------------------------------------------------------------------------
    IAnalogMonitor Interface
    -------------------------------------------------------------------------*/
    void update( const float val, const size_t time_us );
  };
}  // namespace Orbit::Monitor

#endif  /* !ORBIT_CURRENT_MONITOR_HPP */
