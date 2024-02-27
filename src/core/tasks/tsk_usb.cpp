/******************************************************************************
 *  File Name:
 *    tsk_usb.cpp
 *
 *  Description:
 *    USB task implementation
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/hw/orbit_usb.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_usb.hpp>

#if defined( EMBEDDED )
#include <tusb.h>
#endif

namespace Orbit::Tasks::USB
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void USBThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Run the USB thread
    -------------------------------------------------------------------------*/
    Orbit::USB::powerUp();
    Orbit::LED::attachUSBActiveListener();

    while ( 1 )
    {
      #if defined( EMBEDDED )
      /*-----------------------------------------------------------------------
      Process high priority USB interrupts
      -----------------------------------------------------------------------*/
      tud_task();

      /*-----------------------------------------------------------------------
      Notify the CDC thread of any pending work RX data that needs processing.
      -----------------------------------------------------------------------*/
      if( tud_cdc_available() )
      {
        Chimera::Thread::sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ), TASK_MSG_CDC_WAKEUP,
                                      Chimera::Thread::TIMEOUT_DONT_WAIT );
      }
      #else
      Chimera::delayMilliseconds( 100 );
      #endif  /* EMBEDDED */
    }
  }
}    // namespace Orbit::Tasks::USB
