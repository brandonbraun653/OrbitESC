/******************************************************************************
 *  File Name:
 *    orbit_gpio.cpp
 *
 *  Description:
 *    Orbit GPIO bus driver
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/exti>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_gpio.hpp>


namespace Orbit::GPIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    Chimera::GPIO::Driver_rPtr gpio = nullptr;

    /*-------------------------------------------------------------------------
    Initialize the LED output enable pin
    -------------------------------------------------------------------------*/
    #if defined( ORBIT_ESC_V2 )
    gpio = Chimera::GPIO::getDriver( IO::Digital::ledOEPort, IO::Digital::ledOEPin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( IO::Digital::ledOEPinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->setState( Chimera::GPIO::State::LOW ) );
    #endif

    /*-------------------------------------------------------------------------
    Intialize the debug output pins
    -------------------------------------------------------------------------*/
    #if defined( ORBIT_ESC_V3 )
    gpio = Chimera::GPIO::getDriver( IO::Digital::dbg1Port, IO::Digital::dbg1Pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( IO::Digital::dbg1PinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->setState( Chimera::GPIO::State::LOW ) );

    gpio = Chimera::GPIO::getDriver( IO::Digital::dbg2Port, IO::Digital::dbg2Pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( IO::Digital::dbg2PinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->setState( Chimera::GPIO::State::LOW ) );
    #endif
  }

}    // namespace Orbit::GPIO
