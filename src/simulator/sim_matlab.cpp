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
#include <Chimera/timer>
#include <ChimeraSim/timer>
#include <cstring>
#include <src/control/foc_data.hpp>
#include <src/control/foc_driver.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/simulator/sim_matlab.hpp>
#include <src/trace/orbit_trace.hpp>
#include <lib/ChimeraSim/source/peripherals/timer/sim_chimera_timer.hpp>

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
    float va; /**< Phase A voltage in Volts */
    float vb; /**< Phase B voltage in Volts */
    float vc; /**< Phase C voltage in Volts */
    float ia; /**< Phase A current in Amps */
    float ib; /**< Phase B current in Amps */
    float ic; /**< Phase C current in Amps */
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
    float armed;          /**< System should be armed */
    float engaged;        /**< System should be engaged */
    float speed_ref_rpm;  /**< Speed reference in rpm */
    float supply_voltage; /**< Power supply voltage in Volts */
    float sim_time_us;    /**< Simulation time in microseconds */

    ControlData()
    {
      armed          = 0.0f;
      engaged        = 0.0f;
      speed_ref_rpm  = 0.0f;
      supply_voltage = 0.0f;
      sim_time_us    = 0.0f;
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
    Parse the received data
    -------------------------------------------------------------------------*/

    /*-------------------------------------------------------------------------
    Step the motor control loops
    -------------------------------------------------------------------------*/
    // TODO Next steps: Need to re-figure out the control loop sequence and
    // how to run it in a way that decouples general thread execution from
    // the matlab simulation. Likely need to refresh on FOC control and cleaning
    // up the code.

    /*-------------------------------------------------------------------------
    Send results back to Matlab simulation
    -------------------------------------------------------------------------*/
    Orbit::Trace::traceAlphaBetaCommands( Control::foc_ireg_state.va_cmd, Control::foc_ireg_state.vb_cmd, Chimera::micros() );
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

    if( ( new_cmd.sim_time_us > 0.0f ) && Chimera::Timer::Sim::isExternalTimeSourceActive() )
    {
      Chimera::Timer::Sim::updateExternalTime( static_cast<size_t>( new_cmd.sim_time_us ) );
    }

    /*-------------------------------------------------------------------------
    Change motor controller state
    -------------------------------------------------------------------------*/
    if( static_cast<bool>( new_cmd.armed ) && !static_cast<bool>( s_prev_cmd.armed ) )
    {
      Control::FOC::sendSystemEvent( Control::EventId::ARM );
    }

    if( static_cast<bool>( new_cmd.engaged ) && !static_cast<bool>( s_prev_cmd.engaged ) )
    {
      Control::FOC::sendSystemEvent( Control::EventId::ENGAGE );
    }

    /*-------------------------------------------------------------------------
    Inject ADC measurements for system parameters
    -------------------------------------------------------------------------*/
    // Supply voltage

    /*-------------------------------------------------------------------------
    Set target references
    -------------------------------------------------------------------------*/
    // Speed reference

    s_prev_cmd = new_cmd;
  }

  void escControlConnectionCallback( Orbit::Sim::TCP::Server &server, Orbit::Sim::TCP::ConnectionState state )
  {
    ( void )server;

    switch( state )
    {
      case Orbit::Sim::TCP::ConnectionState::Connected:
        LOG_INFO( "ESC control client connected" );

        Orbit::Motor::Sense::reset();
        Orbit::Motor::Drive::reset();
        s_prev_cmd = {};

        Orbit::Control::foc_motor_state = {};
        Orbit::Control::foc_ireg_state.iqPID.resetState();
        Orbit::Control::foc_ireg_state.idPID.resetState();
        Orbit::Control::foc_ireg_state.iqRef     = 0.0f;
        Orbit::Control::foc_ireg_state.idRef     = 0.0f;
        Orbit::Control::foc_ireg_state.va_cmd    = 0.0f;
        Orbit::Control::foc_ireg_state.vb_cmd    = 0.0f;
        Orbit::Control::foc_ireg_state.vq        = 0.0f;
        Orbit::Control::foc_ireg_state.vd        = 0.0f;
        Orbit::Control::foc_ireg_state.vq_mod    = 0.0f;
        Orbit::Control::foc_ireg_state.vd_mod    = 0.0f;
        Orbit::Control::foc_ireg_state.max_drive = 0.0f;

        Orbit::Control::FOC::sendSystemEvent( Orbit::Control::EventId::DISABLE );
        Chimera::Timer::Sim::enableExternalTimeSource( 0U );
        break;

      case Orbit::Sim::TCP::ConnectionState::Disconnected:
        LOG_INFO( "ESC control client disconnected" );
        Orbit::Control::FOC::sendSystemEvent( Orbit::Control::EventId::DISABLE );
        Chimera::Timer::Sim::disableExternalTimeSource();
        break;
    }
  }
}    // namespace Orbit::Sim::Matlab
