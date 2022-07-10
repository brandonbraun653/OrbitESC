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
#include <Chimera/gpio>
#include <src/core/hw/orbit_gpio.hpp>
#include <src/config/bsp/board_map.hpp>


namespace Orbit::GPIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    using namespace Chimera::GPIO;

    /*-------------------------------------------------------------------------
    User Button
    -------------------------------------------------------------------------*/
    PinInit cfg;

    cfg.clear();
    cfg.threaded  = true;
    cfg.validity  = true;
    cfg.alternate = Chimera::GPIO::Alternate::NONE;
    cfg.drive     = Chimera::GPIO::Drive::INPUT;
    cfg.pin       = IO::GPIO::pinButton;
    cfg.port      = IO::GPIO::portButton;
    cfg.pull      = Chimera::GPIO::Pull::PULL_UP;
    cfg.state     = Chimera::GPIO::State::HIGH;

    auto pin = getDriver( cfg.port, cfg.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg ) );
  }

}    // namespace Orbit::GPIO
