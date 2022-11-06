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
#include <Aurora/memory>
#include <Aurora/logging>
#include <Chimera/thread>
#include <Chimera/can>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_hwm.hpp>
#include <src/core/runtime/adc_runtime.hpp>
#include <src/core/runtime/can_runtime.hpp>


#include <Chimera/i2c>
#include <src/config/bsp/board_map.hpp>

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
    Orbit::CAN::initRuntime();
    Orbit::ADC::initRuntime();

    /*-------------------------------------------------------------------------
    Run the HWM thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while( 1 )
    {
      /*---------------------------------------------------------------------
      Process hardware drivers
      ---------------------------------------------------------------------*/
      Orbit::CAN::processCANBus();
      Orbit::ADC::processADC();

      /*---------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      ---------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::HWM
