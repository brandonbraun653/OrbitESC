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

    auto cfg = Aurora::Flash::EEPROM::DeviceConfig();
    cfg.clear();
    cfg.deviceAddress = 0x53;
    cfg.i2cChannel    = IO::I2C::channel;
    cfg.whichChip     = Aurora::Flash::EEPROM::Chip::AT24C02;

    auto eeprom = Aurora::Flash::EEPROM::Driver();
    eeprom.configure( cfg );

    uint8_t raw_data = 0x24;
    size_t last_write = Chimera::millis();


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

      if ( ( Chimera::millis() - last_write ) > 25 )
      {
        eeprom.write( 0x10, &raw_data, sizeof( raw_data ) );
        last_write = Chimera::millis();
      }

      /*---------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      ---------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::HWM
