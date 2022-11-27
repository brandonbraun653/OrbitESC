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


  void VoltageMonitor::update( const float val, const size_t time_us )
  {
    // TODO: Need to trip only after a sustained period above/below threshold

    if ( mEngageState != EngageState::ACTIVE )
    {
      return;
    }

    if ( val < mThreshMin )
    {
      mTripState = TripState::EXCEED_LOWER;
      return;
    }

    if ( val > mThreshMax )
    {
      mTripState = TripState::EXCEED_UPPER;
      return;
    }
  }

}    // namespace Orbit::Monitor
