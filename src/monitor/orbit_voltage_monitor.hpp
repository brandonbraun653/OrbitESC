/******************************************************************************
 *  File Name:
 *    orbit_voltage_monitor.hpp
 *
 *  Description:
 *    Voltage monitoring solution
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_VOLTAGE_MONITOR_HPP
#define ORBIT_VOLTAGE_MONITOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_analog_monitor.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct VMonConfig
  {
    // Filter parameters
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class VoltageMonitor : public virtual IAnalogMonitor
  {
  public:
    VoltageMonitor();
    ~VoltageMonitor();

    /*-------------------------------------------------------------------------
    IAnalogMonitor Interface
    -------------------------------------------------------------------------*/
    void setEngageState( const EngageState state );
    void setThresholds( const float min, const float max );
    TripState tripped();
    void update( const float val, const size_t time_us );
  };
}  // namespace Orbit::Monitor

#endif  /* !ORBIT_VOLTAGE_MONITOR_HPP */
