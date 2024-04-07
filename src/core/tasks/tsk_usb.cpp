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

#if defined( EMBEDDED )
#include <tusb.h>
#endif

namespace Orbit::Tasks::USB
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t TX_BUF_SZ     = 512;
  static constexpr size_t RX_BUF_SZ     = 128;
  static constexpr size_t TX_ISR_BUF_SZ = 4096;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

#if defined( SIMULATOR )
  static void isrLock();
  static void isrUnlock();
#endif /* SIMULATOR */


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

#if defined( SIMULATOR )
  static Chimera::Thread::Mutex      s_usb_isr_mutex;
  static etl::function_fv<isrLock>   usb_isr_lock;
  static etl::function_fv<isrUnlock> usb_isr_unlock;
#else
  static etl::function_fv<Orbit::USB::enableInterrupts>  usb_isr_lock;
  static etl::function_fv<Orbit::USB::disableInterrupts> usb_isr_unlock;
#endif /* SIMULATOR */
  static etl::circular_buffer<uint8_t, TX_BUF_SZ>       s_tx_buffer;
  static etl::circular_buffer<uint8_t, RX_BUF_SZ>       s_rx_buffer;
  static etl::queue_spsc_locked<uint8_t, TX_ISR_BUF_SZ> s_tx_isr_buffer{ usb_isr_lock, usb_isr_unlock };


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
    usb_serial->init( Endpoint::COM_ENDPOINT, &s_rx_buffer, &s_tx_buffer, &s_tx_isr_buffer );
    usb_serial->open( {} );

    /* Begin the HW initialization sequence */
    Orbit::USB::attach();

    while ( 1 )
    {
      #if defined( EMBEDDED )
      /*-----------------------------------------------------------------------
      Process high priority USB interrupts
      -----------------------------------------------------------------------*/
      tud_task();


      usb_serial->process();
      #else
      Chimera::delayMilliseconds( 100 );
      #endif  /* EMBEDDED */
    }
  }

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

#if defined( SIMULATOR )
  static void isrLock()
  {
    s_usb_isr_mutex.lock();
  }


  static void isrUnlock()
  {
    s_usb_isr_mutex.unlock();
  }
#endif /* SIMULATOR */
}    // namespace Orbit::Tasks::USB
