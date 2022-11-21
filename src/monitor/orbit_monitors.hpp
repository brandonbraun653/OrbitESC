/******************************************************************************
 *  File Name:
 *    orbit_monitors.hpp
 *
 *  Description:
 *    System controller for the monitoring solutions used on Orbit
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_MONITORS_HPP
#define ORBIT_MONITORS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <src/monitor/orbit_voltage_monitor.hpp>
#include <src/monitor/orbit_current_monitor.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern VoltageMonitor                             VBusMonitor;  /**< Power supply monitor */
  extern CurrentMonitor                             IPhAMonitor;  /**< Phase A current monitor */
  extern CurrentMonitor                             IPhBMonitor;  /**< Phase B current monitor */
  extern CurrentMonitor                             IPhCMonitor;  /**< Phase C current monitor */
  extern const etl::array<IAnalogMonitor *const, 4> MonitorArray; /**< All available monitors */

  /*-----------------------------------------------------------------------------
  Public Functions
  -----------------------------------------------------------------------------*/
  /**
   * @brief Prepares monitors for work
   */
  void initialize();

}    // namespace Orbit::Monitor

#endif /* !ORBIT_MONITORS_HPP */
