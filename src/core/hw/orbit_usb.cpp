/******************************************************************************
 *  File Name:
 *    orbit_usb.cpp
 *
 *  Description:
 *    USB driver implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/

#include <Aurora/logging>
#include <Chimera/gpio>
#include <Chimera/thread>
#include <Chimera/system>
#include <src/config/bsp/board_map.hpp>
#include <Thor/lld/interface/inc/interrupt>
#include <src/core/hw/orbit_tusb.h>

#include <tusb.h>

namespace Orbit::USB
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t LOG_BUF_SIZE = 512;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static char                   s_log_buffer[ LOG_BUF_SIZE ];
  static Chimera::Thread::Mutex s_format_lock;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Configure GPIO
    -------------------------------------------------------------------------*/
    Chimera::GPIO::Driver_rPtr pin = nullptr;

    /* DM Signal */
    pin = Chimera::GPIO::getDriver( IO::USB::dmPort, IO::USB::dmPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USB::dmPinInit ) );

    /* DP Signal */
    pin = Chimera::GPIO::getDriver( IO::USB::dpPort, IO::USB::dpPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USB::dpPinInit ) );

    /* VBUS Signal */
    pin = Chimera::GPIO::getDriver( IO::USB::vbusPort, IO::USB::vbusPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USB::vbusPinInit ) );

    /* Enumerate Signal */
    pin = Chimera::GPIO::getDriver( IO::USB::enumPort, IO::USB::enumPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USB::enumPinInit ) );

    /*-------------------------------------------------------------------------
    Configure USB clocks
    -------------------------------------------------------------------------*/
    tusb_configure_clocks();

    /*-------------------------------------------------------------------------
    Configure Interrupts
    -------------------------------------------------------------------------*/
    const IRQn_Type irq[] = { OTG_HS_EP1_OUT_IRQn, OTG_HS_EP1_IN_IRQn, OTG_HS_WKUP_IRQn, OTG_HS_IRQn };

    for ( auto i = 0u; i < ARRAY_COUNT( irq ); i++ )
    {
      Thor::LLD::INT::setPriority( irq[ i ], Thor::LLD::INT::USB_IT_PREEMPT_PRIORITY, 0u );
      Thor::LLD::INT::enableIRQ( irq[ i ] );
    }

    /*-------------------------------------------------------------------------
    Initialize TinyUSB
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( true == tusb_init() );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->setState( Chimera::GPIO::State::HIGH ) );
  }
}    // namespace Orbit::USB


/*-----------------------------------------------------------------------------
TinyUSB functionality that uses our C++ infrastructure
-----------------------------------------------------------------------------*/
extern "C"
{
  /**
   * @brief Logs a message from the USB driver to the system logger
   *
   * @param format  The format string
   * @param ...     Variable arguments
   * @return int    Number of bytes written
   */
  int orbit_usb_printf( const char *format, ... )
  {
    using namespace Orbit::USB;
    using namespace Aurora::Logging;

    if ( Chimera::System::inISR() )
    {
      return 0;
    }

    // TODO BMB: Evaluate if this is necessary
    // Chimera::Thread::LockGuard _lock( s_format_lock );

    memset( s_log_buffer, 0, LOG_BUF_SIZE );
    va_list argptr;
    va_start( argptr, format );
    const int write_size = npf_vsnprintf( s_log_buffer, LOG_BUF_SIZE, format, argptr );
    va_end( argptr );

    if ( write_size > 0 )
    {
      getRootSink()->log( Level::LVL_DEBUG, s_log_buffer, static_cast<size_t>( write_size ) );
      return write_size;
    }
    else
    {
      return 0;
    }
  }


  /**
   * @brief Invoked when device is mounted (configured)
   * @return void
   */
  void tud_mount_cb( void )
  {
    Chimera::insert_debug_breakpoint();
  }


  /**
   * @brief Invoked when the device is unmounted
   * @return void
   */
  void tud_umount_cb( void )
  {
    Chimera::insert_debug_breakpoint();
  }

} /* extern "C" */
