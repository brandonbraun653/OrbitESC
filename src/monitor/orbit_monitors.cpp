/******************************************************************************
 *  File Name:
 *    orbit_monitors.cpp
 *
 *  Description:
 *    Implementation of high level monitor details
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_monitors.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  VoltageMonitor                             VBusMonitor;
  CurrentMonitor                             IPhAMonitor;
  CurrentMonitor                             IPhBMonitor;
  CurrentMonitor                             IPhCMonitor;
  const etl::array<IAnalogMonitor *const, 4> MonitorArray = { &VBusMonitor, &IPhAMonitor, &IPhBMonitor, &IPhCMonitor };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initialize()
  {

  }
}    // namespace Orbit::Monitor
