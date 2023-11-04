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
#include <Chimera/function>
#include <Thor/lld/interface/inc/interrupt>
#include <etl/vector.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_tusb.h>
#include <src/core/hw/orbit_usb_intf.h>
#include <src/monitor/debug/segger_modules_intf.h>

#include <tusb.h>

namespace Orbit::USB
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static etl::vector<Chimera::Function::Opaque, 2> s_connect_callbacks;
  static etl::vector<Chimera::Function::Opaque, 2> s_disconnect_callbacks;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize Module Data
    -------------------------------------------------------------------------*/
    s_connect_callbacks.clear();
    s_disconnect_callbacks.clear();

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
    Thor::LLD::INT::setPriority( OTG_HS_IRQn, 2u, 0u );
    Thor::LLD::INT::enableIRQ( OTG_HS_IRQn );

    /*-------------------------------------------------------------------------
    Initialize TinyUSB
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( true == tusb_init() );
    OrbitMonitorRecordEvent_TUSB( TUSB_Init );
  }


  bool onConnect( Chimera::Function::Opaque &&callback )
  {
    if( s_connect_callbacks.full() )
    {
      return false;
    }

    s_connect_callbacks.push_back( std::move( callback ) );
    return true;
  }


  bool onDisconnect( Chimera::Function::Opaque &&callback )
  {
    if( s_disconnect_callbacks.full() )
    {
      return false;
    }

    s_disconnect_callbacks.push_back( std::move( callback ) );
    return true;
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

    /*-------------------------------------------------------------------------
    Invoke the user callbacks
    -------------------------------------------------------------------------*/
    for( auto &cb : Orbit::USB::s_connect_callbacks )
    {
      cb();
    }
  }


  /**
   * @brief Invoked when the device is unmounted
   * @return void
   */
  void tud_umount_cb( void )
  {
    OrbitMonitorRecordEvent_TUSB( TUSB_Unmount );

    /*-------------------------------------------------------------------------
    Invoke the user callbacks
    -------------------------------------------------------------------------*/
    for( auto &cb : Orbit::USB::s_disconnect_callbacks )
    {
      cb();
    }
  }
} /* extern "C" */
