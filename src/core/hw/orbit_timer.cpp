/******************************************************************************
 *  File Name:
 *    orbit_timer.cpp
 *
 *  Description:
 *    Orbit TIMER bus driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/timer>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_timer.hpp>


namespace Orbit::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    // NOT PRODUCTION CODE
    using namespace Chimera::Timer;

    Chimera::Timer::PWM::Driver *pwm =
        Chimera::Timer::getDriver<Chimera::Timer::PWM::Driver>( Instance::TIMER16 );
    RT_HARD_ASSERT( pwm );

    Chimera::Timer::PWM::DriverConfig cfg;
    cfg.clear();
    cfg.polarity               = Chimera::Timer::PWM::Polarity::ACTIVE_HIGH;
    cfg.safeIOLevel            = Chimera::GPIO::State::HIGH;
    cfg.coreCfg.instance       = Instance::TIMER16;
    cfg.outputChannel          = Channel::CHANNEL_1;
    cfg.dutyCycle              = 50.0f;
    cfg.frequency              = 10000.0f;

    auto cfg_result = pwm->init( cfg );
    pwm->enableOutput();
  }

}  // namespace Orbit::TIMER
