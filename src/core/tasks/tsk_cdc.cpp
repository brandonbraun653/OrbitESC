/******************************************************************************
 *  File Name:
 *    tsk_usb.cpp
 *
 *  Description:
 *    USB CDC task implementation to support serial communication
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_usb.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <etl/circular_buffer.h>

namespace Orbit::Tasks::USB::CDC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static etl::circular_buffer<uint8_t, 1024> s_tx_buffer;
  static etl::circular_buffer<uint8_t, 1024> s_rx_buffer;


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
    USBSerial *usb = getUSBSerialDriver();
    usb->init( 0, &s_rx_buffer, &s_tx_buffer );

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Wait for the USB task to notify us there is work to do
      -----------------------------------------------------------------------*/
      Chimera::Thread::this_thread::pendTaskMsg( TASK_MSG_CDC_WAKEUP, 5u * TIMEOUT_1MS );
      usb->process();
    }
  }
}    // namespace Orbit::Tasks::USB::CDC
