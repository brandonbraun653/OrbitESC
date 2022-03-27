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

    auto i2c = Chimera::I2C::getDriver( IO::I2C::channel );
    uint8_t raw_data = 0x24;


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

      i2c->write( 0x53, &raw_data, sizeof( raw_data ) );

      /*---------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      ---------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::HWM
