/******************************************************************************
 *  File Name:
 *    tsk_hwm.cpp
 *
 *  Description:
 *    Hardware manager task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/memory>
#include <Chimera/can>
#include <Chimera/thread>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/runtime/adc_runtime.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_hwm.hpp>

namespace Orbit::Tasks::HWM
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void HWMThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize the HWM drivers
    -------------------------------------------------------------------------*/
    Orbit::ADC::initRuntime();

    /*-------------------------------------------------------------------------
    Run the HWM thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Process hardware drivers
      -----------------------------------------------------------------------*/
      Orbit::LED::sendUpdate();
      Orbit::ADC::processADC();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::HWM
