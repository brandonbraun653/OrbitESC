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

namespace Orbit::Tasks::USB::CDC
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t TX_BUF_SZ = 512;
  static constexpr size_t RX_BUF_SZ = 128;

  static constexpr size_t TX_ISR_BUF_SZ = 1024;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static etl::circular_buffer<uint8_t, TX_BUF_SZ> s_tx_buffer;
  static etl::circular_buffer<uint8_t, RX_BUF_SZ> s_rx_buffer;
  static auto s_isr_buffer= etl::queue_spsc_locked<uint8_t, TX_ISR_BUF_SZ>( etl::function_fv<Orbit::USB::disableInterrupts>(), etl::function_fv<Orbit::USB::enableInterrupts>());

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
    usb->init( 0, &s_rx_buffer, &s_tx_buffer, &s_isr_buffer );

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Poll or wait for someone to notify us there is work to do.
      -----------------------------------------------------------------------*/
      Chimera::Thread::this_thread::pendTaskMsg( TASK_MSG_CDC_WAKEUP, 10u * TIMEOUT_1MS );
      usb->process();
    }
  }
}    // namespace Orbit::Tasks::USB::CDC
