/******************************************************************************
 *  File Name:
 *    orbit_trace.hpp
 *
 *  Description:
 *    OrbitESC-specific trace implementation for alpha/beta command tracing
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_HPP
#define ORBIT_TRACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_interface.hpp>
#include <src/trace/trace_types.hpp>
#include <Aurora/logging>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  OrbitESC Trace Data Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Alpha/beta voltage commands for FOC control
   * @note Matlab requires a single data type
   */
  struct AlphaBetaCommands
  {
    float alpha_cmd;       // Alpha-axis voltage command (V)
    float beta_cmd;        // Beta-axis voltage command (V)
    float timestamp_us;    // Timestamp in microseconds
  };

  /*---------------------------------------------------------------------------
  OrbitESC Trace Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the OrbitESC trace system
   *
   * Sets up trace callbacks for alpha/beta command tracing to Matlab simulation.
   * Uses appropriate communication method based on build configuration.
   */
  void initialize();

  /**
   * @brief Trace alpha/beta voltage commands
   *
   * @param alpha_cmd Alpha-axis voltage command (V)
   * @param beta_cmd Beta-axis voltage command (V)
   * @param timestamp_us Timestamp in microseconds
   */
  void traceAlphaBetaCommands( float alpha_cmd, float beta_cmd, uint32_t timestamp_us );

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_HPP */
