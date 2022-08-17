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

    auto cfg = Aurora::Flash::EEPROM::DeviceConfig();
    cfg.clear();
    cfg.deviceAddress = 0x53;
    cfg.i2cChannel    = IO::I2C::channel;
    cfg.whichChip     = Aurora::Flash::EEPROM::Chip::AT24C02;

    auto eeprom = Aurora::Flash::EEPROM::Driver();
    eeprom.configure( cfg );

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
      // Orbit::ADC::processADC();

      // if ( ( Chimera::millis() - last_write ) > 1000 )
      // {
      //   if( rw_flag )
      //   {
      //     eeprom.write( rw_address, hello, strlen( hello ) );
      //     eeprom.await( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE, Chimera::Thread::TIMEOUT_10MS );
      //     rw_flag = false;
      //   }
      //   else
      //   {
      //     rw_flag = true;

      //     memset( read_buf, 0, sizeof( read_buf ) );
      //     eeprom.read( rw_address, read_buf, strlen( hello ) );
      //     eeprom.await( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE, Chimera::Thread::TIMEOUT_10MS );

      //     LOG_INFO( "EEPROM says: %s\r\n", read_buf );
      //   }
      //   last_write = Chimera::millis();
      // }

      /*---------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      ---------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::HWM
