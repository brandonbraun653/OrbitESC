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
#include <src/simulator/sim_matlab.hpp>
#include <src/control/foc_driver.hpp>
#include <src/control/foc_data.hpp>
#include <src/trace/orbit_trace.hpp>

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

    ControlData()
    {
      armed          = 0.0f;
      engaged        = 0.0f;
      speed_ref_rpm  = 0.0f;
      supply_voltage = 0.0f;
    }
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

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

    /*-------------------------------------------------------------------------
    Send results back to Matlab simulation
    -------------------------------------------------------------------------*/
    Orbit::Trace::traceAlphaBetaCommands( Control::foc_ireg_state.va_cmd, Control::foc_ireg_state.vb_cmd, Chimera::micros() );
  }

  void escControlCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size )
  {
    static ControlData prev_cmd = {};

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

    /*-------------------------------------------------------------------------
    Change motor controller state
    -------------------------------------------------------------------------*/
    if( static_cast<bool>( new_cmd.armed ) && !static_cast<bool>( prev_cmd.armed ) )
    {
      Control::FOC::sendSystemEvent( Control::EventId::ARM );
    }

    if( static_cast<bool>( new_cmd.engaged ) && !static_cast<bool>( prev_cmd.engaged ) )
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

    prev_cmd = new_cmd;
  }
}    // namespace Orbit::Sim::Matlab
