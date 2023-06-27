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

#include <src/core/hw/orbit_adc.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctl.hpp>
#include <src/monitor/orbit_monitors.hpp>
#include <src/core/hw/orbit_motor.hpp>


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

    Chimera::delayMilliseconds( 1000 );
    Motor::powerUp();


    /*-------------------------------------------------------------------------
    Run the CTL thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::CTL
