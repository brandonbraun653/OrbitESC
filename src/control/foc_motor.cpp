/******************************************************************************
 *  File Name:
 *    foc_motor.cpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Motor Control. This module is responsible
 *    for implementing the realtime control loops required for FOC. It does not
 *    handle mode transitions or other high level control logic.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/control/foc_motor.hpp>
#include <src/core/hw/orbit_motor.hpp>

namespace Orbit::Control::Field
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ControlState
  {
    Mode ctl_mode; /**< Current control mode */
  };


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile ControlState s_state; /**< Current state of the control loop */
  static volatile Chimera::GPIO::Driver_rPtr s_dbg_pin; /**< Debug pin for timing measurements */

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Executes a single cycle of the current control algorithm
   * @note  This function is called from the ADC DMA ISR
   *
   * This function consumes the latest ADC samples, runs the control algorithm,
   * and updates the PWM outputs for the next cycle.
   */
  static void isr_current_control_loop()
  {
    Orbit::Motor::SenseData sense_data;
    Orbit::Motor::getSenseData( sense_data );

    /*-------------------------------------------------------------------------
    Set the debug pin low to indicate the end of the control loop
    -------------------------------------------------------------------------*/
    s_dbg_pin->setState( Chimera::GPIO::State::LOW );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize the control state
    -------------------------------------------------------------------------*/
    s_state.ctl_mode = Mode::DISABLED;

    /*-------------------------------------------------------------------------
    Get a reference to the debug pin. This is used for timing measurements.
    Works in concert with orbit_motor_sense.cpp.
    -------------------------------------------------------------------------*/
    s_dbg_pin = Chimera::GPIO::getDriver( Orbit::IO::Digital::dbg1Port, Orbit::IO::Digital::dbg1Pin );

    /*-------------------------------------------------------------------------
    Map the current control function to the ADC DMA ISR
    -------------------------------------------------------------------------*/
    Chimera::Function::Opaque func = Chimera::Function::Opaque::create<isr_current_control_loop>();
    Orbit::Motor::setSenseCallback( func );
  }


  bool setControlMode( const Mode mode )
  {
    // TODO
    return false;
  }


  Mode getControlMode()
  {
    return s_state.ctl_mode;
  }
}    // namespace Orbit::Control::Field
