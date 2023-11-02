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
#include <src/core/hw/orbit_usb.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_usb.hpp>
#include <tusb.h>

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

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Process high priority USB interrupts
      -----------------------------------------------------------------------*/
      tud_task();
      tud_cdc_write_flush();

      /*-----------------------------------------------------------------------
      Notify the CDC thread of any pending work
      -----------------------------------------------------------------------*/
      if( tud_cdc_available() )
      {
        Chimera::Thread::sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ), TASK_MSG_CDC_WAKEUP,
                                      Chimera::Thread::TIMEOUT_DONT_WAIT );
      }
    }
  }
}    // namespace Orbit::Tasks::USB
