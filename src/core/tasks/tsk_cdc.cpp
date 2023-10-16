/******************************************************************************
 *  File Name:
 *    tsk_usb.cpp
 *
 *  Description:
 *    USB task implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_usb.hpp>
#include <tusb.h>

namespace Orbit::Tasks::USB::CDC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void USBCDCThread( void *arg )
  {
    static constexpr size_t PERIOD_MS = 5;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Run the CDC thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while ( 1 )
    {
      // connected() check for DTR bit
      // Most but not all terminal client set this when making connection
      // if ( tud_cdc_connected() )
      {
        // There are data available
        while ( tud_cdc_available() )
        {
          uint8_t buf[ 64 ];

          // read and echo back
          uint32_t count = tud_cdc_read( buf, sizeof( buf ) );
          ( void )count;

          // Echo back
          // Note: Skip echo by commenting out write() and write_flush()
          // for throughput test e.g
          //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
          tud_cdc_write( buf, count );
        }

        tud_cdc_write_flush();
      }

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::USB::CDC
