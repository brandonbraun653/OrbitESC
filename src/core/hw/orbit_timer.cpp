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
  Public Data
  ---------------------------------------------------------------------------*/
  Chimera::Timer::PWM::Driver PWMDriver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Configure the GPIO
    -------------------------------------------------------------------------*/
    Chimera::GPIO::PinInit pin_cfg;

    pin_cfg.clear();
    pin_cfg.validity  = true;
    pin_cfg.threaded  = true;
    pin_cfg.alternate = Chimera::GPIO::Alternate::TIM16_CH1;
    pin_cfg.drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL;
    pin_cfg.pin       = IO::GPIO::pinHeartbeat;
    pin_cfg.port      = IO::GPIO::portHeartbeat;
    pin_cfg.pull      = Chimera::GPIO::Pull::NO_PULL;
    pin_cfg.state     = Chimera::GPIO::State::LOW;

    auto pin = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Configure the PWM timer
    -------------------------------------------------------------------------*/
    Chimera::Timer::PWM::DriverConfig cfg;
    cfg.clear();
    cfg.polarity               = Chimera::Timer::PWM::Polarity::ACTIVE_HIGH;
    cfg.safeIOLevel            = Chimera::GPIO::State::HIGH;
    cfg.coreCfg.instance       = Chimera::Timer::Instance::TIMER16;
    cfg.coreCfg.baseFreq       = 1'000'000.0f;
    cfg.coreCfg.clockSource    = Chimera::Clock::Bus::SYSCLK;
    cfg.outputChannel          = Chimera::Timer::Channel::CHANNEL_1;
    cfg.dutyCycle              = 50.0f;
    cfg.frequency              = 10000.0f;

    auto cfg_result = PWMDriver.init( cfg );
    PWMDriver.enableOutput();
  }

}  // namespace Orbit::TIMER
