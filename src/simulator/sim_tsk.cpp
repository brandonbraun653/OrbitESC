/******************************************************************************
 *  File Name:
 *    sim_tsk.cpp
 *
 *  Description:
 *    Simulator thread for the test harness and simulation environment
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/simulator/sim_tsk.hpp>
#include <src/simulator/sim_adc.hpp>
#include <src/simulator/sim_tcp_server.hpp>

namespace Orbit::Tasks::SIM
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void SIMThread( void *arg )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize TCP servers for Matlab communication
    -------------------------------------------------------------------------*/
    // Create main TCP server for critical system data (port 55001)
    Orbit::Sim::TCP::ServerConfig main_config;
    main_config.port           = 55001;
    main_config.rx_buffer_size = 1024;
    main_config.tx_buffer_size = 1024;
    main_config.rx_callback    = []( Orbit::Sim::TCP::Server &server, const void *data, size_t size ) {
      // Echo received data back to Matlab
      if( server.isClientConnected() )
      {
        server.sendData( data, size );
      }
    };

    auto main_server = Orbit::Sim::TCP::ServerManager::getInstance().createServer( main_config );
    if( !main_server )
    {
      // Log error but continue startup
    }

    // Create trace/logging server for plotting data (port 55002)
    Orbit::Sim::TCP::ServerConfig trace_config;
    trace_config.port           = 55002;
    trace_config.rx_buffer_size = 4096;    // Larger buffer for trace data
    trace_config.tx_buffer_size = 4096;
    trace_config.rx_callback    = []( Orbit::Sim::TCP::Server &server, const void *data, size_t size ) {
      // Process trace data from Matlab
      // TODO: Implement trace data processing logic
    };

    auto trace_server = Orbit::Sim::TCP::ServerManager::getInstance().createServer( trace_config );
    if( !trace_server )
    {
      // Log error but continue startup
    }

    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Run simulation events
      -----------------------------------------------------------------------*/
      Orbit::Sim::ADC::triggerInstrumentationADC();

      Chimera::delayMilliseconds( PERIOD_MS );
    }
  }
}    // namespace Orbit::Tasks::SIM
