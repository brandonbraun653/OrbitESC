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
#include <Thor/lld/interface/inc/interrupt>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_tusb.h>
#include <src/core/hw/orbit_usb_intf.h>
#include <src/monitor/debug/segger_modules_intf.h>

#include <tusb.h>

namespace Orbit::USB
{
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

    for( auto i = 0u; i < ARRAY_COUNT( irq ); i++ )
    {
      Thor::LLD::INT::setPriority( irq[ i ], Thor::LLD::INT::USB_IT_PREEMPT_PRIORITY, 0u );
      Thor::LLD::INT::enableIRQ( irq[ i ] );
    }

    /*-------------------------------------------------------------------------
    Initialize TinyUSB
    -------------------------------------------------------------------------*/
    OrbitSetDPPullupState( false );
    RT_HARD_ASSERT( true == tusb_init() );
    OrbitMonitorRecordEvent_TUSB( TUSB_Init );
    Chimera::delayMilliseconds( 100 );
    OrbitSetDPPullupState( true );
  }
}    // namespace Orbit::USB


/*-----------------------------------------------------------------------------
TinyUSB functionality that uses our C++ infrastructure
-----------------------------------------------------------------------------*/
extern "C"
{
  /**
   * @brief Invoked when device is mounted (configured)
   * @return void
   */
  void tud_mount_cb( void )
  {
    OrbitMonitorRecordEvent_TUSB( TUSB_Mount );
  }


  /**
   * @brief Invoked when the device is unmounted
   * @return void
   */
  void tud_umount_cb( void )
  {
    OrbitMonitorRecordEvent_TUSB( TUSB_Unmount );
  }


  void OrbitSetDPPullupState( const bool state )
  {
    using namespace Orbit;

    Chimera::GPIO::Driver_rPtr pin = Chimera::GPIO::getDriver( IO::USB::dpPort, IO::USB::dpPin );
    RT_HARD_ASSERT( pin );

    if( state )
    {
      OrbitMonitorRecordEvent_TUSB( TUSB_DP_PULLUP_ENABLE );
      pin->setState( Chimera::GPIO::State::HIGH );
    }
    else
    {
      OrbitMonitorRecordEvent_TUSB( TUSB_DP_PULLUP_DISABLE );
      pin->setState( Chimera::GPIO::State::LOW );
    }
  }
} /* extern "C" */
