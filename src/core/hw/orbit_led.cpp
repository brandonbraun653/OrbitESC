/******************************************************************************
 *  File Name:
 *    orbit_led.cpp
 *
 *  Description:
 *    LED driver implementation for Orbit ESC
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/spi>
#include <Chimera/thread>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_led.hpp>


namespace Orbit::LED
{
  /*---------------------------------------------------------------------------
  Assertions
  ---------------------------------------------------------------------------*/
  static_assert( NUM_LEDS < 8, "Invalid number of LEDs" );

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Thread::Mutex s_data_lock;     /* Threading protection */
  static uint8_t                s_led_state;     /* Cached LED state */
  static uint8_t                s_prv_led_state; /* Last LED state */

#if defined( ORBIT_ESC_V3 )
  static Chimera::GPIO::Driver_rPtr s_led_pins[ NUM_LEDS ]; /* LED pins */
#elif defined( ORBIT_ESC_V2 ) || defined( ORBIT_ESC_V1 )
  static Chimera::SPI::Driver_rPtr  s_spi_driver; /* SPI driver */
  static Chimera::GPIO::Driver_rPtr s_led_cs_pin; /* LED chip select */
  static Chimera::GPIO::Driver_rPtr s_led_oe_pin; /* LED chip select */
#endif

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/
#if defined( ORBIT_ESC_V2 ) || defined( ORBIT_ESC_V1 )
  static void powerUpHW_V1_V2()
  {
    /*-------------------------------------------------------------------------
    Get pointers to the required hardware drivers
    -------------------------------------------------------------------------*/
    s_spi_driver = Chimera::SPI::getDriver( IO::SPI::spiChannel );
    s_led_cs_pin = Chimera::GPIO::getDriver( IO::SPI::ledCSPort, IO::SPI::ledCSPin );
    RT_HARD_ASSERT( s_spi_driver );
    RT_HARD_ASSERT( s_led_cs_pin );

#if defined( ORBIT_ESC_V2 )
    s_led_oe_pin = Chimera::GPIO::getDriver( IO::Digital::ledOEPort, IO::Digital::ledOEPin );
    RT_HARD_ASSERT( s_led_oe_pin );
#endif
  }


  static void updateLEDs_V1_V2()
  {
    /*-------------------------------------------------------------------------
    Send the update to the shift register
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _spiLock( *s_spi_driver );
    const uint8_t              led_cache = s_led_state;

#if defined( ORBIT_ESC_V2 )
    s_led_oe_pin->setState( Chimera::GPIO::State::HIGH );
#endif

    s_spi_driver->assignChipSelect( s_led_cs_pin );
    s_spi_driver->setChipSelectControlMode( Chimera::SPI::CSMode::MANUAL );
    s_spi_driver->setChipSelect( Chimera::GPIO::State::LOW );
    s_spi_driver->writeBytes( &led_cache, sizeof( led_cache ) );
    s_spi_driver->await( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE, Chimera::Thread::TIMEOUT_10MS );
    s_spi_driver->setChipSelect( Chimera::GPIO::State::HIGH );
    s_spi_driver->assignChipSelect( nullptr );

#if defined( ORBIT_ESC_V2 )
    s_led_oe_pin->setState( Chimera::GPIO::State::LOW );
#endif

    /*-------------------------------------------------------------------------
    Update the cache for the next comparison
    -------------------------------------------------------------------------*/
    s_prv_led_state = led_cache;
  }
#endif /* ORBIT_ESC_V2 || ORBIT_ESC_V1 */

#if defined( ORBIT_ESC_V3 )
  static void powerUpHW_V3()
  {
    /*-------------------------------------------------------------------------
    Fault LED
    -------------------------------------------------------------------------*/
    s_led_pins[ FAULT_POS ] = Chimera::GPIO::getDriver( IO::Digital::ledFaultPort, IO::Digital::ledFaultPin );
    RT_HARD_ASSERT( s_led_pins[ FAULT_POS ] );
    RT_HARD_ASSERT( s_led_pins[ FAULT_POS ]->init( IO::Digital::ledFaultPinInit ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Armed LED 
    -------------------------------------------------------------------------*/
    s_led_pins[ ARMED_POS ] = Chimera::GPIO::getDriver( IO::Digital::ledArmedPort, IO::Digital::ledArmedPin );
    RT_HARD_ASSERT( s_led_pins[ ARMED_POS ] );
    RT_HARD_ASSERT( s_led_pins[ ARMED_POS ]->init( IO::Digital::ledArmedPinInit ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Heartbeat LED
    -------------------------------------------------------------------------*/
    s_led_pins[ HEARTBEAT_POS ] = Chimera::GPIO::getDriver( IO::Digital::ledHeartbeatPort, IO::Digital::ledHeartbeatPin );
    RT_HARD_ASSERT( s_led_pins[ HEARTBEAT_POS ] );
    RT_HARD_ASSERT( s_led_pins[ HEARTBEAT_POS ]->init( IO::Digital::ledHeartbeatPinInit ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    CAN Active LED  
    -------------------------------------------------------------------------*/
    s_led_pins[ CAN_ACTIVE_POS ] = Chimera::GPIO::getDriver( IO::Digital::ledCANActivePort, IO::Digital::ledCANActivePin );
    RT_HARD_ASSERT( s_led_pins[ CAN_ACTIVE_POS ] );
    RT_HARD_ASSERT( s_led_pins[ CAN_ACTIVE_POS ]->init( IO::Digital::ledCANActivePinInit ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    USB Active LED
    -------------------------------------------------------------------------*/
    s_led_pins[ USB_ACTIVE_POS ] = Chimera::GPIO::getDriver( IO::Digital::ledUSBActivePort, IO::Digital::ledUSBActivePin );
    RT_HARD_ASSERT( s_led_pins[ USB_ACTIVE_POS ] );
    RT_HARD_ASSERT( s_led_pins[ USB_ACTIVE_POS ]->init( IO::Digital::ledUSBActivePinInit ) == Chimera::Status::OK );
  }


  static void updateLEDs_V3()
  {
    constexpr Chimera::GPIO::State LED_ON  = Chimera::GPIO::State::HIGH;
    constexpr Chimera::GPIO::State LED_OFF = Chimera::GPIO::State::LOW;

    for ( size_t x = 0; x < ARRAY_COUNT( s_led_pins ); x++ )
    {
      if ( !s_led_pins[ x ] )
      {
        continue;
      }

      if ( s_led_state & ( 1 << x ) )
      {
        s_led_pins[ x ]->setState( LED_ON );
      }
      else
      {
        s_led_pins[ x ]->setState( LED_OFF );
      }
    }
  }
#endif /* ORBIT_ESC_V3 */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Power up the required hardware
    -------------------------------------------------------------------------*/
#if defined( ORBIT_ESC_V2 ) || defined( ORBIT_ESC_V1 )
    powerUpHW_V1_V2();
#elif defined( ORBIT_ESC_V3 )
    powerUpHW_V3();
#endif

    /*-------------------------------------------------------------------------
    Reset the module data
    -------------------------------------------------------------------------*/
    s_data_lock.unlock();
    s_led_state     = 0x00;
    s_prv_led_state = 0xFF;

    /*-------------------------------------------------------------------------
    Update the LED output to idle
    -------------------------------------------------------------------------*/
    sendUpdate();
  }


  void sendUpdate()
  {
    /*-------------------------------------------------------------------------
    Only send the update if the data has changed
    -------------------------------------------------------------------------*/
    if ( s_led_state == s_prv_led_state )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Do HW specific update
    -------------------------------------------------------------------------*/
#if defined( ORBIT_ESC_V2 ) || defined( ORBIT_ESC_V1 )
    updateLEDs_V1_V2();
#elif defined( ORBIT_ESC_V3 )
    updateLEDs_V3();
#endif
  }


  void setChannel( const Channel channel )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    if ( channel < Channel::NUM_OPTIONS )
    {
      s_led_state |= ( channel & ALL_LED_MSK );
    }
  }


  void clearChannel( const Channel channel )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    if ( channel < Channel::NUM_OPTIONS )
    {
      s_led_state &= ~channel;
    }
  }


  void toggleChannel( const Channel channel )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    if ( channel < Channel::NUM_OPTIONS )
    {
      s_led_state ^= channel;
    }
  }
}    // namespace Orbit::LED
