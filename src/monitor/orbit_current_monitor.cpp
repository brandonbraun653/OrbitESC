/******************************************************************************
 *  File Name:
 *    orbit_current_monitor.cpp
 *
 *  Description:
 *    Implementation of a generic current monitor on Orbit
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_current_monitor.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  CurrentMonitor::CurrentMonitor()
  {
  }

  CurrentMonitor::~CurrentMonitor()
  {
  }


  void CurrentMonitor::setEngageState( const EngageState state )
  {
  }


  void CurrentMonitor::setThresholds( const float min, const float max )
  {
  }


  TripState CurrentMonitor::tripped()
  {
    return TripState::NOT_TRIPPED;
  }


  void CurrentMonitor::update( const float val, const size_t time_us )
  {
  }

}    // namespace Orbit::Monitor
