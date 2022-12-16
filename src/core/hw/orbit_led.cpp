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
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Thread::Mutex     s_data_lock;     /* Threading protection */
  static Chimera::SPI::Driver_rPtr  s_spi_driver;    /* SPI driver */
  static Chimera::GPIO::Driver_rPtr s_led_cs_pin;    /* LED chip select */
  static Chimera::GPIO::Driver_rPtr s_led_oe_pin;    /* LED chip select */
  static uint8_t                    s_led_state;     /* Cached LED state */
  static uint8_t                    s_prv_led_state; /* Last LED state */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
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


  void setBits( const uint8_t mask )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    s_led_state = ( s_led_state | mask ) & ALL_LED_MSK;
  }


  void clrBits( const uint8_t mask )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    s_led_state &= ~mask;
  }


  uint8_t getBits()
  {
    return s_led_state;
  }


  void setChannel( const Channel channel )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    if ( channel < Channel::NUM_OPTIONS )
    {
      s_led_state |= ( channel & ALL_LED_MSK );
    }
  }


  void clrChannel( const Channel channel )
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
