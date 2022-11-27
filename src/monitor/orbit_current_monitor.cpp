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


  void CurrentMonitor::update( const float val, const size_t time_us )
  {
    // TODO BMB: Need to do something a little more intelligent, like integrating
    // TODO BMB: sustained energy dissipation to allow for transients that won't
    // TODO BMB: blow the MOSFETs immediately.

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
