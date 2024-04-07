/******************************************************************************
 *  File Name:
 *    tsk_usb.cpp
 *
 *  Description:
 *    USB CDC task implementation to support serial communication
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <etl/circular_buffer.h>
#include <etl/function.h>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/hw/orbit_usb.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_usb.hpp>

#if defined( EMBEDDED )
#include <tusb.h>
#endif

namespace Orbit::Tasks::USB::CDC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void USBCDCThread( void *arg )
  {
    using namespace Orbit::Serial;
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Run the CDC thread
    -------------------------------------------------------------------------*/
    USBSerial *const usb = getUSBSerialDriver();

    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Poll or wait for someone to notify us there is work to do.
      -----------------------------------------------------------------------*/
      Chimera::Thread::this_thread::pendTaskMsg( TASK_MSG_CDC_WAKEUP, 5u * TIMEOUT_1MS );
      if( tud_mounted() )
      {
        //usb->process();
      }
    }
  }
}    // namespace Orbit::Tasks::USB::CDC
