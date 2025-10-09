/******************************************************************************
 *  File Name:
 *    trace_examples.hpp
 *
 *  Description:
 *    Usage examples and integration patterns for the trace module
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TRACE_EXAMPLES_HPP
#define ORBIT_TRACE_EXAMPLES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_interface.hpp>
#include <src/trace/trace_types.hpp>
#include <Aurora/logging>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Example Data Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Example motor current measurements structure
   */
  struct MotorCurrentData
  {
    float phase_a_current;    // Phase A current in Amps
    float phase_b_current;    // Phase B current in Amps
    float phase_c_current;    // Phase C current in Amps
    float dc_bus_current;     // DC bus current in Amps
  };

  /**
   * @brief Example control system data structure
   */
  struct ControlSystemData
  {
    float error_signal;        // Control error
    float output_signal;       // Control output
    float reference_signal;    // Reference signal
    float integral_term;       // Integral term
    float derivative_term;     // Derivative term
  };

  /**
   * @brief Example system status structure
   */
  struct SystemStatusData
  {
    uint32_t system_state;    // System state flags
    float    temperature;     // System temperature in Celsius
    float    voltage;         // System voltage in Volts
    float    current;         // System current in Amps
    uint32_t error_flags;     // Error flags
  };

  /*---------------------------------------------------------------------------
  Example Callback Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Example callback for embedded serial communication
   *
   * This callback sends trace data over the serial interface using
   * the existing serial communication system.
   */
  bool embeddedSerialCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

  /**
   * @brief Example callback for simulator TCP communication
   *
   * This callback sends trace data over TCP to Matlab using
   * the simulator's TCP server.
   */
  bool simulatorTcpCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

  /**
   * @brief Example callback for logging trace data
   *
   * This callback logs trace data to the console for debugging.
   */
  bool loggingCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us );

  /*---------------------------------------------------------------------------
  Example Usage Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Example of how to initialize the trace system
   *
   * This function shows how to set up the trace system with appropriate
   * callbacks for the current environment.
   */
  void initializeTraceSystem();

  /**
   * @brief Example of how to send motor current measurements
   *
   * @param current_data Motor current measurements to trace
   */
  void traceMotorCurrents( const MotorCurrentData &current_data );

  /**
   * @brief Example of how to send control system data
   *
   * @param control_data Control system data to trace
   */
  void traceControlSystem( const ControlSystemData &control_data );

  /**
   * @brief Example of how to send system status
   *
   * @param status_data System status data to trace
   */
  void traceSystemStatus( const SystemStatusData &status_data );

  /**
   * @brief Example of how to send custom user data
   *
   * @param custom_data Pointer to custom data
   * @param size Size of custom data
   */
  void traceCustomData( const void *custom_data, size_t size );

  /*---------------------------------------------------------------------------
  Integration Patterns
  ---------------------------------------------------------------------------*/
  /**
   * @brief Example of integrating trace calls into control loops
   *
   * This function shows how to add trace calls to control loops
   * without significantly impacting performance.
   */
  void exampleControlLoopIntegration();

  /**
   * @brief Example of conditional tracing based on system state
   *
   * This function shows how to conditionally enable/disable
   * tracing based on system state or configuration.
   */
  void exampleConditionalTracing();

  /**
   * @brief Example of rate-limited tracing
   *
   * This function shows how to use the built-in rate limiting
   * feature to reduce trace data volume.
   */
  void exampleRateLimitedTracing();

}    // namespace Orbit::Trace

#endif /* !ORBIT_TRACE_EXAMPLES_HPP */
