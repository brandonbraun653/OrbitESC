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
#include <src/control/current_control.hpp>
#include <src/control/foc_data.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/speed_control.hpp>
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
    Set the debug pin high to indicate the start of the control loop
    -------------------------------------------------------------------------*/
    s_dbg_pin->setState( Chimera::GPIO::State::HIGH );

    static const float    deg2rad    = 0.0174533f;
    static volatile float start_time = Chimera::micros() / 1e6f;
    static volatile float dt         = 0.0f;
    static volatile float theta      = 0.0f;

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    s_speed_ctrl_timer.ackISR();
    // if ( !s_state.isrControlActive )
    // {
    //   start_time = Chimera::micros() / 1e6f;
    //   dt         = 0.0f;
    //   return;
    // }

    // !TESTING
    const float ramp_time = ( Chimera::micros() / 1e6f ) - start_time;
    if ( ramp_time < 2.0f )
    {
      dt = ramp_time;
    }
    else
    {
      //ramp_drive = 0.25;
    }
    // else if ( !s_state.switchToClosedLoop )
    // {
    //   s_state.switchToClosedLoop = true;
    //   s_state.iLoop.vq           = 0.0f;
    //   s_state.iLoop.vd           = 0.0f;
    //   s_state.iLoop.idPID.resetState();
    //   s_state.iLoop.iqPID.resetState();
    // }

    /*-----------------------------------------------------------------------------
    Limit the ramp rate of theta so that it can't cross more than one sector
    -----------------------------------------------------------------------------*/
    // float dTheta = ( Data::SysControl.rampCtrlSecondOrderTerm * deg2rad * dt * dt ) +
    //                ( Data::SysControl.rampCtrlFirstOrderTerm * deg2rad * dt );

    float dTheta = ( 15.0f * deg2rad * dt * dt ) + ( 1.5f * deg2rad * dt );

    dTheta = Control::Math::clamp( dTheta, 0.0f, DEG_TO_RAD( 59.9f ) );
    theta += dTheta;

    /*-----------------------------------------------------------------------------
    Limit the ramp rate of theta so that it can't cross more than one sector
    -----------------------------------------------------------------------------*/
    if ( theta > 6.283185f )
    {
      theta -= 6.283185f;
    }

    foc_motor_state.thetaEst = theta;

    // // TODO: Update these with the speed control PI loops
    // s_state.iLoop.iqRef = 0.00002f;    // Simulink model was using this in startup?
    // s_state.iLoop.idRef = 0.0f;

    // !TESTING

    // runOuterLoopSpeedControl();

    /*-------------------------------------------------------------------------
    Push the latest streaming parameters into the transmission buffer
    -------------------------------------------------------------------------*/
    // const uint32_t timestamp = Chimera::micros();
    // publishPhaseCurrents( timestamp );
    // publishPhaseCommands( timestamp );
    // publishStateEstimates( timestamp );

    /*-------------------------------------------------------------------------
    Set the debug pin low to indicate the start of the control loop
    -------------------------------------------------------------------------*/
    s_dbg_pin->setState( Chimera::GPIO::State::LOW );
  }



// TODO: Copied over from orbit_motor.cpp. Need to figure out what to do with it.
  // static inline void runOuterLoopSpeedControl()
  // {
  //   static constexpr float KSpeed = 1.0f;    // TODO: Tune this/make it configurable

  //   static const float irp_percalc = ( Data::SysControl.statorPWMFreq / Data::SysControl.speedCtrlUpdateFreq );

  //   SpeedPosObserverState *pAlg = &s_state.sObserve;

  //   /*-------------------------------------------------------------------------
  //   Calculate the unfiltered speed estimate
  //   -------------------------------------------------------------------------*/
  //   pAlg->omega_est       = pAlg->acc_theta_delta * KSpeed;
  //   pAlg->acc_theta_delta = 0.0f;

  //   /*-------------------------------------------------------------------------
  //   Generate filtered speed estimate with first order low pass filter
  //   -------------------------------------------------------------------------*/
  //   pAlg->omega_filtered = pAlg->lpf_alpha * pAlg->omega_est + ( 1.0f - pAlg->lpf_alpha ) * pAlg->omega_filtered;

  //   /*-------------------------------------------------------------------------
  //   Compensate the theta estimate for the filtered speed
  //   -------------------------------------------------------------------------*/
  //   pAlg->theta_comp = pAlg->theta_est + pAlg->omega_filtered;

  //   /*-------------------------------------------------------------------------
  //   Update the LPF gains based on the current speed
  //   -------------------------------------------------------------------------*/
  //   // TODO: Don't do this until we have a stable speed estimate
  //   // pAlg->lpf_alpha = pAlg->omega_filtered * ( Control::Math::M_PI_F / irp_percalc );
  //   // s_state.iObserve.phase[ 0 ].lpf_alpha = pAlg->lpf_alpha;
  //   // s_state.iObserve.phase[ 1 ].lpf_alpha = pAlg->lpf_alpha;
  // }


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
