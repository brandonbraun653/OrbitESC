/******************************************************************************
 *  File Name:
 *    speed_control.cpp
 *
 *  Description:
 *    Outer loop speed controller implementation for a FOC motor
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <src/config/bsp/board_map.hpp>
#include <src/config/orbit_esc_cfg.hpp>
#include <src/control/foc_data.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/foc_observer.hpp>
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_motor.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/interface/inc/timer>
#endif /* EMBEDDED */

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* SEGGER_SYS_VIEW */


namespace Orbit::Control::Speed
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Trigger::Master     s_speed_ctrl_timer; /**< Trigger for the speed control loop */
  static volatile Chimera::GPIO::Driver_rPtr s_dbg_pin;          /**< Debug pin for timing measurements */
  static volatile Mode                       s_ctl_mode;         /**< Current control mode */
  static Control::Math::PID                  s_speed_pid;        /**< Speed controller PID */

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Callback to process the speed controller within a timer ISR
   */
  static void timer_isr_speed_controller()
  {
    using namespace Orbit::Motor;

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    s_speed_ctrl_timer.ackISR();

    /*-------------------------------------------------------------------------
    Run the PID controller to generate a new Iq reference
    -------------------------------------------------------------------------*/
    Observer::Output observer = Observer::estimates();

    // TODO BMB: Replace this with a parameter
    const float motor_poles = 7.0f;

    // TODO BMB: Replace this with a runtime variable
    s_speed_pid.SetPoint = 1000.0f; // RPM

    foc_ireg_state.iqRef = s_speed_pid.run( observer.omega_elec / motor_poles );
    foc_ireg_state.idRef = 0.0f;

    // TODO BMB: I'm worried about there being an output discontinuity here with
    // TODO BMB: the commanded iqRef. Need to think about how to handle this.
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize module state
    -------------------------------------------------------------------------*/
    s_ctl_mode = Mode::DISABLED;

    /*-------------------------------------------------------------------------
    Initialize the debug pin
    -------------------------------------------------------------------------*/
    s_dbg_pin = Chimera::GPIO::getDriver( Orbit::IO::Digital::dbg2Port, Orbit::IO::Digital::dbg2Pin );
    s_dbg_pin->setState( Chimera::GPIO::State::LOW );

    /*-------------------------------------------------------------------------
    Initialize the speed controller PID
    -------------------------------------------------------------------------*/
    s_speed_pid.init();
    s_speed_pid.OutMaxLimit = 10.0f;
    s_speed_pid.OutMinLimit = -10.0f;
    s_speed_pid.setTunings( 15.0f, 0.1f, 0.0f, 1.0f / Data::SysControl.statorPWMFreq );
    s_speed_pid.resetState();

    /*-------------------------------------------------------------------------
    Configure the Speed control outer loop update timer
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = Orbit::Data::SysControl.speedCtrlUpdateFreq;
    trig_cfg.isrCallback            = Chimera::Function::Opaque::create<timer_isr_speed_controller>();
    trig_cfg.coreConfig.instance    = Orbit::IO::Timer::SpeedControl;
    trig_cfg.coreConfig.baseFreq    = 100'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == s_speed_ctrl_timer.init( trig_cfg ) );
    s_speed_ctrl_timer.enable();
  }

  void powerDn()
  {
  }


  void synchronize( const float omega )
  {
    s_speed_pid.resetState();
    s_speed_pid.SetPoint = omega;
  }

  bool setControlMode( const Mode mode )
  {
    // TODO
    return false;
  }


  Mode getControlMode()
  {
    return s_ctl_mode;
  }
}    // namespace Orbit::Control::Speed
