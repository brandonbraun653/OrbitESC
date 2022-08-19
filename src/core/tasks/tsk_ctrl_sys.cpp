/******************************************************************************
 *  File Name:
 *    tsk_sys_ctrl.cpp
 *
 *  Description:
 *    Control system task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/thread>
#include <Aurora/logging>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctrl_sys.hpp>
#include <src/control/foc_driver.hpp>
#include <src/core/hw/drv8301.hpp>

namespace Orbit::Tasks::CTRLSYS
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void CTRLSYSThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize the CTRLSYS drivers
    -------------------------------------------------------------------------*/
    Orbit::Control::FOCConfig cfg;

    cfg.adcSource = Chimera::ADC::Peripheral::ADC_0;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_SUPPLY_VOLTAGE ]  = BoostXL::adc_to_bus_voltage;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_PHASE_A_CURRENT ] = BoostXL::adc_to_phase_current;
    cfg.txfrFuncs[ Control::ADC_CH_MOTOR_PHASE_B_CURRENT ] = BoostXL::adc_to_phase_current;

    Orbit::Control::MotorParameters params;
    params.Rs = 0.01f;
    params.Ls = 380.0f * 1e-3f;

    Orbit::Control::FOCDriver.initialize( cfg, params );

    /*-------------------------------------------------------------------------
    Run the CTRLSYS thread
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
}    // namespace Orbit::Tasks::CTRLSYS
