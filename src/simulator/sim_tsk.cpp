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
#include <Aurora/logging>
#include <Chimera/common>
#include <Chimera/thread>
#include <src/control/foc_data.hpp>
#include <src/control/foc_driver.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/core/tasks.hpp>
#include <src/simulator/sim_adc.hpp>
#include <src/simulator/sim_matlab.hpp>
#include <src/simulator/sim_tcp_server.hpp>
#include <src/simulator/sim_tsk.hpp>

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
    // Create main TCP server for critical system data
    Orbit::Sim::TCP::ServerConfig main_config;
    main_config.port           = Sim::TCP::MOTOR_SIMULATION_PORT;
    main_config.rx_buffer_size = 1024;
    main_config.tx_buffer_size = 1024;
    main_config.rx_callback    = Orbit::Sim::Matlab::motorSimulationCallback;

    auto main_server =
        Orbit::Sim::TCP::ServerManager::getInstance().createServer(
            main_config );
    LOG_WARN_IF( !main_server, "Failed to create main server" );

    // Create ESC control TCP server for state control commands
    Orbit::Sim::TCP::ServerConfig esc_config;
    esc_config.port           = Sim::TCP::ESC_CONTROL_PORT;
    esc_config.rx_buffer_size = 1024;
    esc_config.tx_buffer_size = 1024;
    esc_config.rx_callback    = Orbit::Sim::Matlab::escControlCallback;
    esc_config.connection_callback =
        Orbit::Sim::Matlab::escControlConnectionCallback;

    auto esc_server =
        Orbit::Sim::TCP::ServerManager::getInstance().createServer(
            esc_config );
    LOG_WARN_IF( !esc_server, "Failed to create ESC control server" );

    while( 1 )
    {
      TaskMsg tsk_msg = TASK_MSG_NUM_OPTIONS;

      /*-----------------------------------------------------------------------
      Process messages from the Matlab control server
      -----------------------------------------------------------------------*/
      if( this_thread::receiveTaskMsg( tsk_msg, PERIOD_MS ) )
      {
        switch( tsk_msg )
        {
          case TASK_MSG_CTRL_ARM:
            Control::FOC::sendSystemEvent( Control::EventId::ARM );
            break;

          case TASK_MSG_CTRL_ENGAGE:
            Control::FOC::sendSystemEvent( Control::EventId::ENGAGE );
            break;

          case TASK_MSG_CTRL_DISABLE:
            Control::FOC::sendSystemEvent( Control::EventId::DISABLE );

            Orbit::Motor::Sense::reset();
            Orbit::Motor::Drive::reset();

            Orbit::Control::foc_motor_state = {};
            Orbit::Control::foc_ireg_state.iqPID.resetState();
            Orbit::Control::foc_ireg_state.idPID.resetState();
            Orbit::Control::foc_ireg_state.iqRef     = 0.0f;
            Orbit::Control::foc_ireg_state.idRef     = 0.0f;
            Orbit::Control::foc_ireg_state.va_cmd    = 0.0f;
            Orbit::Control::foc_ireg_state.vb_cmd    = 0.0f;
            Orbit::Control::foc_ireg_state.vq        = 0.0f;
            Orbit::Control::foc_ireg_state.vd        = 0.0f;
            Orbit::Control::foc_ireg_state.max_drive = 0.0f;
            break;
        }
      }
    }

    /*-----------------------------------------------------------------------
    Run simulation events
    -----------------------------------------------------------------------*/
    Orbit::Sim::ADC::triggerInstrumentationADC();
  }
}    // namespace Orbit::Tasks::SIM
