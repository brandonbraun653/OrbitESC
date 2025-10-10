/******************************************************************************
 *  File Name:
 *    orbit_trace.cpp
 *
 *  Description:
 *    OrbitESC-specific trace implementations
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/orbit_trace.hpp>
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_server.hpp>
#include <src/simulator/sim_tcp_server.hpp>
#include <Chimera/thread>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  OrbitESC Trace Callback Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Embedded serial callback for alpha/beta commands
   */
  static bool embeddedAlphaBetaCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    return false;
  }

  /**
   * @brief Simulator TCP callback for alpha/beta commands
   */
  static bool simulatorAlphaBetaCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
#if defined( SIMULATOR )
    // Get the TCP server manager
    auto &tcp_manager = Orbit::Sim::TCP::ServerManager::getInstance();
    auto  tcp_server  = tcp_manager.getServer( Sim::TCP::MOTOR_SIMULATION_PORT );
    if( !tcp_server )
    {
      LOG_ERROR( "TCP server not available for alpha/beta trace callback" );
      return false;
    }

    // Send the trace data over TCP
    return tcp_server->sendData( data, size );
#else
    // Not available in embedded builds
    return false;
#endif
  }

  /*---------------------------------------------------------------------------
  OrbitESC Trace Implementation
  ---------------------------------------------------------------------------*/
  void initialize()
  {
    // Initialize the trace interface
    auto &trace_interface = getTraceInterface();
    if( !trace_interface.initialize() )
    {
      LOG_ERROR( "Failed to initialize OrbitESC trace interface" );
      return;
    }

    // Register callbacks based on build configuration
#if defined( SIMULATOR )
    // Simulator: Use TCP callback for Matlab communication
    trace_interface.registerCallback( TraceType::MOTOR_VOLTAGE_COMMANDS, simulatorAlphaBetaCallback, 0 );    // 1ms rate
#else
    // Embedded: Use serial callback
    trace_interface.registerCallback( TraceType::MOTOR_VOLTAGE_COMMANDS, embeddedAlphaBetaCallback, 1000 );    // 1ms rate
#endif

    LOG_INFO( "OrbitESC trace system initialized" );
  }

  void traceAlphaBetaCommands( float alpha_cmd, float beta_cmd, uint32_t timestamp_us )
  {
    AlphaBetaCommands cmd_data;
    cmd_data.alpha_cmd    = alpha_cmd;
    cmd_data.beta_cmd     = beta_cmd;
    cmd_data.timestamp_us = timestamp_us;

    getTraceInterface().sendTrace( TraceType::MOTOR_VOLTAGE_COMMANDS, &cmd_data, sizeof( cmd_data ) );
  }

}    // namespace Orbit::Trace
