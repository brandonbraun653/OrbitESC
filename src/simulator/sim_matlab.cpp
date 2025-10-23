/******************************************************************************
 *  File Name:
 *    sim_matlab.cpp
 *
 *  Description:
 *    Matlab simulator callback implementations
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <Chimera/timer>
#include <ChimeraSim/timer>
#include <cstring>
#include <src/control/foc_data.hpp>
#include <src/core/tasks.hpp>
#include <src/simulator/sim_matlab.hpp>
#include <src/simulator/sim_adc.hpp>
#include <src/simulator/sim_observer.hpp>
#include <src/trace/orbit_trace.hpp>
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>

namespace Orbit::Sim::Matlab
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Motor simulation data from Matlab simulation
   * @note Matlab requires a single data type
   */
  struct MotorData
  {
    double omega_rad_s;    /**< Rotor speed in radians per second */
    double elec_angle_rad; /**< Rotor electrical angle in radians */
    double ia;             /**< Phase A current in Amps */
    double ib;             /**< Phase B current in Amps */
    double ic;             /**< Phase C current in Amps */
    double va;             /**< Phase A voltage in Volts */
    double vb;             /**< Phase B voltage in Volts */
    double vc;             /**< Phase C voltage in Volts */
  };

  /**
   * @brief System state control from Matlab simulation
   * @note Matlab requires a single data type
   *
   * These are transitions/events/references that normally come from the
   * flight computer, representing input from a human via some interface
   * like a joystick or a button.
   */
  struct ControlData
  {
    double armed;          /**< System should be armed */
    double engaged;        /**< System should be engaged */
    double speed_ref_rpm;  /**< Speed reference in rpm */
    double supply_voltage; /**< Power supply voltage in Volts */
    double sim_time_sec;   /**< Simulation time in seconds */

    ControlData()
    {
      armed          = 0.0;
      engaged        = 0.0;
      speed_ref_rpm  = 0.0;
      supply_voltage = 0.0;
      sim_time_sec   = 0.0;
    }
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  namespace
  {
    ControlData s_prev_cmd = {};
  }

  void motorSimulationCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size )
  {
    if( !server.isClientConnected() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Parse the received data - handle multiple buffered frames
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( size % sizeof( MotorData ) == 0 );

    const size_t     num_frames = size / sizeof( MotorData );
    const MotorData *frame_data = static_cast<const MotorData *>( data );

    for( size_t i = 0; i < num_frames; ++i )
    {
      const MotorData &new_data = frame_data[ i ];

      /*-------------------------------------------------------------------------
      Inject simulated measurements for system parameters
      -------------------------------------------------------------------------*/
      Orbit::Sim::ADC::setPhaseData( new_data.va, new_data.vb, new_data.vc, new_data.ia, new_data.ib, new_data.ic );
      Orbit::Sim::ADC::triggerMotorSenseADC();

      Orbit::Sim::Control::Observer::inject( new_data.elec_angle_rad, new_data.omega_rad_s );

      /*-------------------------------------------------------------------------
      Step the motor control loops
      -------------------------------------------------------------------------*/
      Orbit::Control::Field::isr_current_control_loop();
      Orbit::Control::Speed::timer_isr_speed_controller();    // Need to call at some sub interval of the motor control loop

      /*-------------------------------------------------------------------------
      Send results back to Matlab simulation
      -------------------------------------------------------------------------*/
      Orbit::Trace::traceAlphaBetaCommands( Orbit::Control::foc_ireg_state.va_cmd, Orbit::Control::foc_ireg_state.vb_cmd,
                                            Chimera::micros() );
    }
  }

  void escControlCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size )
  {
    if( !server.isClientConnected() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Parse the received data
    -------------------------------------------------------------------------*/
    ControlData new_cmd;
    if( size != sizeof( ControlData ) )
    {
      return;
    }

    memcpy( &new_cmd, data, sizeof( ControlData ) );

    // TODO: I may want to move the time update to the motor simulation callback.
    // I think it's possible that Matlab doesn't get a 1-1 data/time correlation and
    // it would be easier to sync the HW signals to the simulation time directly.
    // This control callback is really only used for high level reference changes.

    /*-------------------------------------------------------------------------
    Update the system time
    -------------------------------------------------------------------------*/
    if( ( new_cmd.sim_time_sec > 0.0f ) && Chimera::Timer::Sim::isExternalTimeSourceActive() )
    {
      // LOG_INFO( "Matlab time: %f sec", new_cmd.sim_time_sec );
      Chimera::Timer::Sim::updateExternalTime( static_cast<size_t>( new_cmd.sim_time_sec * 1e6 ) );
    }

    /*-------------------------------------------------------------------------
    Change motor controller state
    -------------------------------------------------------------------------*/
    auto task_id = Orbit::Tasks::getTaskId( Orbit::Tasks::TASK_SIM );
    if( static_cast<bool>( new_cmd.armed ) && !static_cast<bool>( s_prev_cmd.armed ) )
    {
      Chimera::Thread::sendTaskMsg( task_id, Tasks::TASK_MSG_CTRL_ARM, 0 );
    }

    if( static_cast<bool>( new_cmd.engaged ) && !static_cast<bool>( s_prev_cmd.engaged ) )
    {
      Chimera::Thread::sendTaskMsg( task_id, Tasks::TASK_MSG_CTRL_ENGAGE, 0 );
    }

    /*-------------------------------------------------------------------------
    Inject ADC measurements for system parameters
    -------------------------------------------------------------------------*/
    Orbit::Sim::ADC::setDCBusVoltage( new_cmd.supply_voltage );
    Orbit::Sim::ADC::triggerInstrumentationADC();

    /*-------------------------------------------------------------------------
    Set target references
    -------------------------------------------------------------------------*/
    // Speed reference

    s_prev_cmd = new_cmd;
  }

  void escControlConnectionCallback( Orbit::Sim::TCP::Server &server, Orbit::Sim::TCP::ConnectionState state )
  {
    ( void )server;
    auto task_id = Orbit::Tasks::getTaskId( Orbit::Tasks::TASK_SIM );

    switch( state )
    {
      case Orbit::Sim::TCP::ConnectionState::Connected:
        LOG_INFO( "ESC control client connected" );
        Chimera::Thread::sendTaskMsg( task_id, Tasks::TASK_MSG_CTRL_DISABLE, 0 );
        Chimera::Timer::Sim::enableExternalTimeSource( 0U );
        break;

      case Orbit::Sim::TCP::ConnectionState::Disconnected:
        LOG_INFO( "ESC control client disconnected" );
        Chimera::Timer::Sim::disableExternalTimeSource();
        Chimera::Thread::sendTaskMsg( task_id, Tasks::TASK_MSG_CTRL_DISABLE, 0 );
        s_prev_cmd = {};
        break;
    }
  }
}    // namespace Orbit::Sim::Matlab
