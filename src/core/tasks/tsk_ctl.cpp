/******************************************************************************
 *  File Name:
 *    tsk_sys_ctrl.cpp
 *
 *  Description:
 *    Control system task
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <Chimera/thread>
#include <src/control/foc_driver.hpp>
#include <src/core/hw/drv8301.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctl.hpp>
#include <src/monitor/orbit_monitors.hpp>


namespace Orbit::Tasks::CTL
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void CTLThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize the CTL drivers
    -------------------------------------------------------------------------*/
    Orbit::Control::FOCConfig cfg;

    cfg.adcSource = Chimera::ADC::Peripheral::ADC_0;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_SUPPLY_VOLTAGE ]  = ADC::sample2BusVoltage;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_PHASE_A_CURRENT ] = ADC::sample2PhaseCurrent;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_PHASE_C_CURRENT ] = ADC::sample2PhaseCurrent;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_PHASE_B_CURRENT ] = ADC::sample2PhaseCurrent;

    Orbit::Control::MotorParameters params;
    params.Rs = 0.01f;
    params.Ls = 380.0f * 1e-3f;

    Orbit::Control::FOCDriver.initialize( cfg, params );
    Orbit::Control::FOCDriver.calibrate();

    Chimera::delayMilliseconds( 1000 );
    // Orbit::Control::FOCDriver.sendSystemEvent( Orbit::Control::EventId::ARM );
    // Chimera::delayMilliseconds( 1000 );
    // Orbit::Control::FOCDriver.driveTestSignal( 1, 50.0f );

    // Orbit::Control::FOCDriver.mTimerDriver.enableOutput();

    /*-------------------------------------------------------------------------
    Run the CTL thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Run the main FOC loop
      -----------------------------------------------------------------------*/
      Orbit::Control::FOCDriver.run();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::CTL
