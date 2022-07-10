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
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctrl_sys.hpp>
#include <src/control/foc_driver.hpp>

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
    Orbit::Control::FOC foc;
    Orbit::Control::FOCConfig cfg;
    cfg.adcSource = Chimera::ADC::Peripheral::ADC_0;

    foc.initialize( cfg );

    /*-------------------------------------------------------------------------
    Run the CTRLSYS thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while ( 1 )
    {
      /*---------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      ---------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::CTRLSYS
