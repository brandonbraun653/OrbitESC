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
  static Chimera::Thread::Mutex     s_data_lock;    /* Threading protection */
  static Chimera::SPI::Driver_rPtr  s_spi_driver;   /* SPI driver */
  static Chimera::GPIO::Driver_rPtr s_led_cs_pin;   /* LED chip select */
  static uint8_t                    s_led_state;    /* Cached LED state */

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

    /*-------------------------------------------------------------------------
    Reset the module data
    -------------------------------------------------------------------------*/
    s_data_lock.unlock();
    s_led_state = 0x00;

    /*-------------------------------------------------------------------------
    Update the LED output to idle
    -------------------------------------------------------------------------*/
    sendUpdate();
  }


  void sendUpdate()
  {
    uint8_t led_cache = s_led_state;

    s_led_cs_pin->setState( Chimera::GPIO::State::LOW );
    s_spi_driver->writeBytes( &led_cache, sizeof( led_cache ) );
    s_spi_driver->await( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE, Chimera::Thread::TIMEOUT_10MS );
    s_led_cs_pin->setState( Chimera::GPIO::State::HIGH );

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
    if( channel < Channel::NUM_OPTIONS )
    {
      s_led_state |= ( 1u << channel ) & ALL_LED_MSK;
    }
  }


  void clrChannel( const Channel channel )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    if( channel < Channel::NUM_OPTIONS )
    {
      s_led_state &= ~( 1u << channel );
    }
  }


  void toggleChannel( const Channel channel )
  {
    Chimera::Thread::LockGuard _lck( s_data_lock );
    if( channel < Channel::NUM_OPTIONS )
    {
      s_led_state ^= ( 1u << channel );
    }
  }
}    // namespace Orbit::LED
