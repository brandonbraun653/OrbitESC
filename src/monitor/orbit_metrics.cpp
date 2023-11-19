/******************************************************************************
 *  File Name:
 *    orbit_metrics.cpp
 *
 *  Description:
 *    Module for tracking various metrics about the system
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_metrics.hpp>


namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Private Data
  ---------------------------------------------------------------------------*/

  static volatile size_t sDataTXCount; /**< Number of data TX events */


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initMetrics()
  {
    sDataTXCount = 0;
  }


  void putDataTXEvent()
  {
    sDataTXCount++;
  }

}  // namespace Orbit::Monitor
