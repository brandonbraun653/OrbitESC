/******************************************************************************
 *  File Name:
 *    orbit_metrics.hpp
 *
 *  Description:
 *    Module for tracking various metrics about the system
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_METRICS_HPP
#define ORBIT_ESC_METRICS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>


namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initializes the metrics module
   * @return void
   */
  void initMetrics();

  /**
   * @brief Indicate that a data TX event has occurred
   * @return void
   */
  void putDataTXEvent();

  /**
   * @brief Gets the total number of data TX events that have occurred
   * @return size_t
   */
  size_t getDataTXCount();

}    // namespace Orbit::Monitor

#endif /* !ORBIT_ESC_METRICS_HPP */
