/******************************************************************************
 *  File Name:
 *    bootup.cpp
 *
 *  Description:
 *    Device driver power up procedures
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <Chimera/assert>
#include <Chimera/gpio>
#include <Chimera/serial>
#include <etl/circular_buffer.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/bootup.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/hw/orbit_gpio.hpp>
#include <src/core/hw/orbit_i2c.hpp>
#include <src/core/hw/orbit_timer.hpp>
#include <src/core/hw/orbit_usart.hpp>
#include <src/core/tasks.hpp>


namespace Orbit::Boot
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUpSystemDrivers()
  {
    //Orbit::ADC::powerUp();
    Orbit::CAN::powerUp();
    Orbit::GPIO::powerUp();
    Orbit::I2C::powerUp();
    Orbit::TIMER::powerUp();
    Orbit::USART::powerUp();
  }


  void startTasks()
  {
    using namespace Chimera::Thread;

    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_HWM ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
  }

}    // namespace Orbit::Boot
