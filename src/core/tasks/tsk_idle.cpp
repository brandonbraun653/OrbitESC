/******************************************************************************
 *  File Name:
 *    tsk_idle.cpp
 *
 *  Description:
 *    Idle task implementation
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/function>
#include <Chimera/gpio>
#include <Chimera/thread>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/hw/orbit_timer.hpp>
#include <src/core/hw/orbit_usb.hpp>
#include <src/core/tasks.hpp>


namespace Orbit::Tasks::BKD
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float IDLE_FLASH_DELAY       = 100.0f;
  static constexpr float IDLE_HOLD_DELAY        = 450.0f;
  static constexpr float USB_ACTIVE_FLASH_DELAY = 50.0f;
  static constexpr float USB_ACTIVE_HOLD_DELAY  = 225.0f;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static float s_flash_delay;
  static float s_hold_delay;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  static void usb_connect()
  {
    s_flash_delay = USB_ACTIVE_FLASH_DELAY;
    s_hold_delay  = USB_ACTIVE_HOLD_DELAY;
  }


  static void usb_disconnect()
  {
    s_flash_delay = IDLE_FLASH_DELAY;
    s_hold_delay  = IDLE_HOLD_DELAY;
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void IdleThread( void *arg )
  {
    using namespace Chimera::Function;

    /*-------------------------------------------------------------------------
    Initialize module data
    -------------------------------------------------------------------------*/
    s_flash_delay = IDLE_FLASH_DELAY;
    s_hold_delay  = IDLE_HOLD_DELAY;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Register USB connection status callbacks
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Orbit::USB::onConnect( Opaque::create<usb_connect>() ) );
    RT_HARD_ASSERT( Orbit::USB::onDisconnect( Opaque::create<usb_disconnect>() ) );

    /*-------------------------------------------------------------------------
    Get the status/heartbeat pin and flash a quick boot up sequence
    -------------------------------------------------------------------------*/
    for( auto x = 0; x < 8; x++ )
    {
      LED::toggleChannel( LED::Channel::ALL );
      Chimera::delayMilliseconds( 35 );
    }
    LED::clearChannel( LED::Channel::ALL );
    Chimera::delayMilliseconds( 500 );

    /*-------------------------------------------------------------------------
    Main loop
    -------------------------------------------------------------------------*/
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Compute the flash and hold times
      -----------------------------------------------------------------------*/
      const uint32_t fd = static_cast<uint32_t>( s_flash_delay * Data::SysConfig.activityLedScaler );
      const uint32_t hd = static_cast<uint32_t>( s_hold_delay * Data::SysConfig.activityLedScaler );

      /*-----------------------------------------------------------------------
      High Pulse #1
      -----------------------------------------------------------------------*/
      LED::setChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( fd );
      LED::clearChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( fd );

      /*-----------------------------------------------------------------------
      High Pulse #2
      -----------------------------------------------------------------------*/
      LED::setChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( fd );
      LED::clearChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( fd );

      /*-----------------------------------------------------------------------
      Hold longer in the off state
      -----------------------------------------------------------------------*/
      Chimera::delayMilliseconds( hd );
    }
  }
}    // namespace Orbit::Tasks::BKD
