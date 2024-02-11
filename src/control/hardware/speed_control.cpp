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
#include <src/control/hardware/current_control.hpp>
#include <src/control/foc_data.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_motor.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/interface/inc/timer>

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* SEGGER_SYS_VIEW */
#endif /* EMBEDDED */


namespace Orbit::Control::Speed
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Trigger::Master     s_speed_ctrl_timer; /**< Trigger for the speed control loop */
  static volatile Chimera::GPIO::Driver_rPtr s_dbg_pin;          /**< Debug pin for timing measurements */
  static volatile Mode                       s_ctl_mode;         /**< Current control mode */

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
    Push the latest streaming parameters into the transmission buffer
    -------------------------------------------------------------------------*/
    // const uint32_t timestamp = Chimera::micros();
    // publishPhaseCurrents( timestamp );
    // publishPhaseCommands( timestamp );
    // publishStateEstimates( timestamp );
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
