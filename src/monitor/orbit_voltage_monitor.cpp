/******************************************************************************
 *  File Name:
 *    orbit_voltage_monitor.cpp
 *
 *  Description:
 *    Voltage monitor implementation
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_voltage_monitor.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  VoltageMonitor::VoltageMonitor()
  {
  }

  VoltageMonitor::~VoltageMonitor()
  {
  }


  void VoltageMonitor::setEngageState( const EngageState state )
  {
  }


  void VoltageMonitor::setThresholds( const float min, const float max )
  {
  }


  TripState VoltageMonitor::tripped()
  {
    return TripState::NOT_TRIPPED;
  }


  void VoltageMonitor::update( const float val, const size_t time_us )
  {
  }

}    // namespace Orbit::Monitor
