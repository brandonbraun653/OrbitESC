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
#include <etl/circular_buffer.h>
#include <etl/function.h>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/hw/orbit_usb.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_usb.hpp>
#include <src/control/hardware/current_control.hpp>

#if defined( EMBEDDED )
#include <tusb.h>
#endif

namespace Orbit::Tasks::USB
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static etl::circular_buffer<uint8_t, 4096> s_tx_buffer;
  static etl::circular_buffer<uint8_t, 128>  s_rx_buffer;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void USBThread( void *arg )
  {
    using namespace Orbit::Serial;
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize the USB driver and any supporting software
    -------------------------------------------------------------------------*/
    Orbit::USB::powerUp();

    /* Update board LED behavior for USB connection state */
    Orbit::LED::attachUSBActiveListener();

    /* Initialize the serial endpoint for host PC communication */
    USBSerial *const usb_serial = getUSBSerialDriver();
    usb_serial->init( Endpoint::COM_ENDPOINT, &s_rx_buffer, &s_tx_buffer );
    usb_serial->open( {} );

    /* Begin the HW initialization sequence */
    Orbit::USB::attach();

    while ( 1 )
    {
      #if defined( EMBEDDED )
      /*-----------------------------------------------------------------------
      Process high priority USB interrupts
      -----------------------------------------------------------------------*/
      tud_task_ext( Chimera::Thread::TIMEOUT_1MS, false );
      usb_serial->process();

      /*-----------------------------------------------------------------------
      Process lower priority USB data sources
      -----------------------------------------------------------------------*/
      Control::Field::pumpISRDataStream();

      #else
      Chimera::delayMilliseconds( 100 );
      #endif  /* EMBEDDED */
    }
  }

}    // namespace Orbit::Tasks::USB
