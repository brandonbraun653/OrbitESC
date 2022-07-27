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

    PinInit cfg;
    Driver_rPtr pin = nullptr;

    /*-------------------------------------------------------------------------
    Heartbeat/Status LED
    -------------------------------------------------------------------------*/
    cfg.clear();
    cfg.validity  = true;
    cfg.threaded  = true;
    cfg.alternate = Alternate::NONE;
    cfg.drive     = Drive::OUTPUT_PUSH_PULL;
    cfg.pin       = IO::GPIO::pinHeartbeat;
    cfg.port      = IO::GPIO::portHeartbeat;
    cfg.pull      = Pull::NO_PULL;
    cfg.state     = State::LOW;

    pin = getDriver( cfg.port, cfg.pin );
    RT_HARD_ASSERT( pin != nullptr );
    auto result = pin->init( cfg );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg ) );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->setState( State::LOW ) );

    /*-------------------------------------------------------------------------
    User Button
    -------------------------------------------------------------------------*/

    cfg.clear();
    cfg.threaded  = true;
    cfg.validity  = true;
    cfg.alternate = Chimera::GPIO::Alternate::NONE;
    cfg.drive     = Chimera::GPIO::Drive::INPUT;
    cfg.pin       = IO::GPIO::pinButton;
    cfg.port      = IO::GPIO::portButton;
    cfg.pull      = Chimera::GPIO::Pull::PULL_UP;
    cfg.state     = Chimera::GPIO::State::HIGH;

    pin = getDriver( cfg.port, cfg.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg ) );
  }

}    // namespace Orbit::GPIO
