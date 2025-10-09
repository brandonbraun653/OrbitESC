/******************************************************************************
 *  File Name:
 *    trace_examples.cpp
 *
 *  Description:
 *    Implementation of usage examples and integration patterns
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/trace/trace_examples.hpp>
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_server.hpp>
#include <src/simulator/sim_tcp_server.hpp>
#include <Chimera/thread>

namespace Orbit::Trace
{
  /*---------------------------------------------------------------------------
  Example Callback Functions
  ---------------------------------------------------------------------------*/
  bool embeddedSerialCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    // Get the serial port instance
    auto serial_port = Orbit::Serial::Config::getCommandPort();
    if( !serial_port )
    {
      LOG_ERROR( "Serial port not available for trace callback" );
      return false;
    }

    // Send the trace data over serial
    // Note: In a real implementation, you might want to wrap this in a proper message
    int bytes_written = serial_port->write( data, size, 1000 );    // 1 second timeout

    if( bytes_written != static_cast<int>( size ) )
    {
      LOG_WARN( "Failed to send complete trace data: sent %d of %zu bytes", bytes_written, size );
      return false;
    }

    LOG_DEBUG( "Sent trace data for type %d: %zu bytes", static_cast<int>( type ), size );
    return true;
  }

  bool simulatorTcpCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
#if defined( SIMULATOR )
    // Get the TCP server manager
    auto &tcp_manager = Orbit::Sim::TCP::ServerManager::getInstance();

    // Use a specific port for trace data (e.g., 37219)
    auto tcp_server = tcp_manager.getServer( 37219 );
    if( !tcp_server )
    {
      LOG_ERROR( "TCP server not available for trace callback" );
      return false;
    }

    // Send the trace data over TCP
    bool success = tcp_server->sendData( data, size );

    if( !success )
    {
      LOG_WARN( "Failed to send trace data over TCP" );
      return false;
    }

    LOG_DEBUG( "Sent trace data for type %d over TCP: %zu bytes", static_cast<int>( type ), size );
    return true;
#else
    // Not available in embedded builds
    return false;
#endif
  }

  bool loggingCallback( TraceType type, const void *data, size_t size, uint32_t timestamp_us )
  {
    LOG_INFO( "Trace[%d]: %zu bytes at %u us", static_cast<int>( type ), size, timestamp_us );

    // Log first few bytes of data for debugging
    const uint8_t *data_bytes = static_cast<const uint8_t *>( data );
    size_t         log_size   = ( size > 16 ) ? 16 : size;

    char hex_string[ 64 ] = { 0 };
    for( size_t i = 0; i < log_size; ++i )
    {
      snprintf( hex_string + i * 3, 4, "%02X ", data_bytes[ i ] );
    }

    LOG_INFO( "Trace data: %s", hex_string );
    return true;
  }

  /*---------------------------------------------------------------------------
  Example Usage Functions
  ---------------------------------------------------------------------------*/
  void initializeTraceSystem()
  {
    // Initialize the trace interface
    auto &trace_interface = getTraceInterface();
    if( !trace_interface.initialize() )
    {
      LOG_ERROR( "Failed to initialize trace interface" );
      return;
    }

    // Register callbacks based on build configuration
#if defined( SIMULATOR )
    // Simulator: Use TCP callback
    trace_interface.registerCallback( TraceType::MOTOR_CURRENT_MEASUREMENTS, simulatorTcpCallback, 1000 );    // 1ms rate
    trace_interface.registerCallback( TraceType::CONTROL_ERROR_SIGNALS, simulatorTcpCallback, 500 );          // 500us rate
    trace_interface.registerCallback( TraceType::SYSTEM_STATUS, simulatorTcpCallback, 10000 );                // 10ms rate
#else
    // Embedded: Use serial callback
    trace_interface.registerCallback( TraceType::MOTOR_CURRENT_MEASUREMENTS, embeddedSerialCallback, 1000 );
    trace_interface.registerCallback( TraceType::CONTROL_ERROR_SIGNALS, embeddedSerialCallback, 500 );
    trace_interface.registerCallback( TraceType::SYSTEM_STATUS, embeddedSerialCallback, 10000 );
#endif

    // Always register logging callback for debugging
    trace_interface.registerCallback( TraceType::MOTOR_CURRENT_MEASUREMENTS, loggingCallback, 0 );
    trace_interface.registerCallback( TraceType::CONTROL_ERROR_SIGNALS, loggingCallback, 0 );
    trace_interface.registerCallback( TraceType::SYSTEM_STATUS, loggingCallback, 0 );

    LOG_INFO( "Trace system initialized with %zu registered callbacks", trace_interface.getRegistrationCount() );
  }

  void traceMotorCurrents( const MotorCurrentData &current_data )
  {
    // Send motor current measurements
    bool success = sendTrace( TraceType::MOTOR_CURRENT_MEASUREMENTS, &current_data, sizeof( current_data ) );

    if( !success )
    {
      LOG_WARN( "Failed to send motor current trace" );
    }
  }

  void traceControlSystem( const ControlSystemData &control_data )
  {
    // Send control system data
    bool success = sendTrace( TraceType::CONTROL_ERROR_SIGNALS, &control_data, sizeof( control_data ) );

    if( !success )
    {
      LOG_WARN( "Failed to send control system trace" );
    }
  }

  void traceSystemStatus( const SystemStatusData &status_data )
  {
    // Send system status
    bool success = sendTrace( TraceType::SYSTEM_STATUS, &status_data, sizeof( status_data ) );

    if( !success )
    {
      LOG_WARN( "Failed to send system status trace" );
    }
  }

  void traceCustomData( const void *custom_data, size_t size )
  {
    // Send custom user data
    bool success = sendTrace( TraceType::USER_CUSTOM_1, custom_data, size );

    if( !success )
    {
      LOG_WARN( "Failed to send custom data trace" );
    }
  }

  /*---------------------------------------------------------------------------
  Integration Patterns
  ---------------------------------------------------------------------------*/
  void exampleControlLoopIntegration()
  {
    // Example of integrating trace calls into a control loop
    static uint32_t          loop_counter = 0;
    static MotorCurrentData  current_data = { 0.0f, 0.0f, 0.0f, 0.0f };
    static ControlSystemData control_data = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    // Simulate control loop execution
    loop_counter++;

    // Update current measurements (simulated)
    current_data.phase_a_current = 1.0f + 0.1f * sinf( loop_counter * 0.01f );
    current_data.phase_b_current = 1.0f + 0.1f * sinf( loop_counter * 0.01f + 2.094f );
    current_data.phase_c_current = 1.0f + 0.1f * sinf( loop_counter * 0.01f + 4.188f );
    current_data.dc_bus_current  = 2.5f;

    // Update control system data (simulated)
    control_data.error_signal     = 0.1f * sinf( loop_counter * 0.005f );
    control_data.output_signal    = 0.8f * control_data.error_signal;
    control_data.reference_signal = 1.0f;
    control_data.integral_term    = 0.01f * control_data.error_signal;
    control_data.derivative_term  = 0.1f * control_data.error_signal;

    // Send trace data (rate limited by registered callbacks)
    traceMotorCurrents( current_data );
    traceControlSystem( control_data );

    // Send system status every 100 loops
    if( loop_counter % 100 == 0 )
    {
      SystemStatusData status_data;
      status_data.system_state = 0x01;    // Running
      status_data.temperature  = 45.0f + 5.0f * sinf( loop_counter * 0.001f );
      status_data.voltage      = 24.0f;
      status_data.current      = 2.5f;
      status_data.error_flags  = 0x00;    // No errors

      traceSystemStatus( status_data );
    }
  }

  void exampleConditionalTracing()
  {
    // Example of conditional tracing based on system state
    static bool     tracing_enabled = true;
    static uint32_t error_count     = 0;

    // Simulate some system condition
    bool system_healthy = ( error_count < 10 );

    if( system_healthy && !tracing_enabled )
    {
      // Enable tracing when system is healthy
      setTraceEnabled( TraceType::MOTOR_CURRENT_MEASUREMENTS, true );
      setTraceEnabled( TraceType::CONTROL_ERROR_SIGNALS, true );
      tracing_enabled = true;
      LOG_INFO( "Tracing enabled - system healthy" );
    }
    else if( !system_healthy && tracing_enabled )
    {
      // Disable tracing when system has errors
      setTraceEnabled( TraceType::MOTOR_CURRENT_MEASUREMENTS, false );
      setTraceEnabled( TraceType::CONTROL_ERROR_SIGNALS, false );
      tracing_enabled = false;
      LOG_WARN( "Tracing disabled - system errors detected" );
    }

    // Simulate error condition
    if( Chimera::millis() % 5000 == 0 )    // Every 5 seconds
    {
      error_count++;
    }
  }

  void exampleRateLimitedTracing()
  {
    // Example of using different rate limits for different trace types
    static uint32_t last_motor_trace   = 0;
    static uint32_t last_control_trace = 0;
    static uint32_t last_status_trace  = 0;

    uint32_t current_time = Chimera::millis();

    // Motor currents: 1ms rate
    if( current_time - last_motor_trace >= 1 )
    {
      MotorCurrentData current_data = { 1.0f, 1.0f, 1.0f, 2.5f };
      traceMotorCurrents( current_data );
      last_motor_trace = current_time;
    }

    // Control signals: 500us rate
    if( current_time - last_control_trace >= 0.5f )
    {
      ControlSystemData control_data = { 0.1f, 0.08f, 1.0f, 0.001f, 0.01f };
      traceControlSystem( control_data );
      last_control_trace = current_time;
    }

    // System status: 10ms rate
    if( current_time - last_status_trace >= 10 )
    {
      SystemStatusData status_data = { 0x01, 45.0f, 24.0f, 2.5f, 0x00 };
      traceSystemStatus( status_data );
      last_status_trace = current_time;
    }
  }

}    // namespace Orbit::Trace
