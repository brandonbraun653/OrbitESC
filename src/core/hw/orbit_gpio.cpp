/******************************************************************************
 *  File Name:
 *    orbit_gpio.cpp
 *
 *  Description:
 *    Orbit GPIO bus driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/exti>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_gpio.hpp>
#include <src/control/foc_driver.hpp>


namespace Orbit::GPIO
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void eStopISR( void * )
  {
    using namespace Orbit::Control;
    FOCDriver.sendSystemEvent( EventId::EMERGENCY_HALT );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize the emergency stop button
    -------------------------------------------------------------------------*/
    Chimera::Function::vGeneric callback = Chimera::Function::vGeneric::create<eStopISR>();

    auto gpio = Chimera::GPIO::getDriver( IO::Digital::eStopPort, IO::Digital::eStopPin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( IO::Digital::eStopPinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->attachInterrupt( callback, Chimera::EXTI::EdgeTrigger::RISING_EDGE ) );
  }

}    // namespace Orbit::GPIO
