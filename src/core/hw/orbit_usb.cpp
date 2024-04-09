/******************************************************************************
 *  File Name:
 *    orbit_usb.cpp
 *
 *  Description:
 *    USB driver implementation
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/function>
#include <Chimera/gpio>
#include <etl/vector.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/hw/orbit_tusb.h>
#include <src/core/hw/orbit_usb.hpp>
#include <src/monitor/debug/segger_modules_intf.h>

#if defined( EMBEDDED )
#include <tusb.h>
#include <Thor/lld/interface/inc/interrupt>
#endif

namespace Orbit::USB
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  #if defined( EMBEDDED )
  static constexpr IRQn_Type USB_IRQn = OTG_HS_IRQn;
  #endif

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static etl::vector<Chimera::Function::Opaque, 2> s_connect_callbacks;
  static etl::vector<Chimera::Function::Opaque, 2> s_disconnect_callbacks;
  static etl::array<Chimera::Function::Opaque, Orbit::Serial::Endpoint::NUM_ENDPOINTS> s_cdc_rx_complete_callbacks;
  static etl::array<Chimera::Function::Opaque, Orbit::Serial::Endpoint::NUM_ENDPOINTS> s_cdc_tx_complete_callbacks;

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
    s_cdc_rx_complete_callbacks.fill( {} );
    s_cdc_tx_complete_callbacks.fill( {} );

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

    #if defined( EMBEDDED )
    /*-------------------------------------------------------------------------
    Configure USB clocks
    -------------------------------------------------------------------------*/
    tusb_configure_clocks();

    /*-------------------------------------------------------------------------
    Configure Interrupts
    -------------------------------------------------------------------------*/
    Thor::LLD::INT::setPriority( USB_IRQn, 1u, 0u );
    Thor::LLD::INT::enableIRQ( USB_IRQn );
    #endif  /* EMBEDDED */
  }


  void attach()
  {
    #if defined( EMBEDDED )
    /*-------------------------------------------------------------------------
    Initialize TinyUSB
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( true == tusb_init() );
    OrbitMonitorRecordEvent_TUSB( TUSB_Init );
    #endif
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


  void disableInterrupts()
  {
    #if defined( EMBEDDED )
    //Thor::LLD::INT::disableIRQ( USB_IRQn );
    #endif
  }


  void enableInterrupts()
  {
    #if defined( EMBEDDED )
    //Thor::LLD::INT::enableIRQ( USB_IRQn );
    #endif
  }


  void onCDCRXComplete( const uint8_t itf, Chimera::Function::Opaque &&callback )
  {
    RT_DBG_ASSERT( itf < s_cdc_rx_complete_callbacks.max_size() );
    s_cdc_rx_complete_callbacks[ itf ] = std::move( callback );
  }


  void onCDCTXComplete( const uint8_t itf, Chimera::Function::Opaque &&callback )
  {
    RT_DBG_ASSERT( itf < s_cdc_tx_complete_callbacks.max_size() );
    s_cdc_tx_complete_callbacks[ itf ] = std::move( callback );
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


  /**
   * @brief Implements the CDC receive complete callback
   *
   * @param itf The interface number that completed the reception
   * @return void
   */
  void tud_cdc_rx_cb( uint8_t itf )
  {
    RT_DBG_ASSERT( itf < Orbit::USB::s_cdc_rx_complete_callbacks.max_size() );
    Orbit::USB::s_cdc_rx_complete_callbacks[ itf ].call_if();
  }


  /**
   * @brief Implements the CDC transmit complete callback
   *
   * @param itf  The interface number that completed the transmission
   * @return void
   */
  void tud_cdc_tx_complete_cb( uint8_t itf )
  {
    RT_DBG_ASSERT( itf < Orbit::USB::s_cdc_tx_complete_callbacks.max_size() );
    Orbit::USB::s_cdc_tx_complete_callbacks[ itf ].call_if();
  }
} /* extern "C" */
