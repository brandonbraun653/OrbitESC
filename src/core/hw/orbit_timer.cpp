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
  Chimera::Timer::Trigger::Master ADCTrigger;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    using namespace Chimera::GPIO;

    /*-------------------------------------------------------------------------
    Configuration Table
    -------------------------------------------------------------------------*/
    const PinInit cfg[] = { { // Heartbeat LED
                              .alternate = Chimera::GPIO::Alternate::TIM16_CH1,
                              .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                              .pin       = IO::GPIO::pinHeartbeat,
                              .port      = IO::GPIO::portHeartbeat,
                              .pull      = Chimera::GPIO::Pull::NO_PULL,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true },
                            { // Inverter High Side Phase A
                              .alternate = Chimera::GPIO::Alternate::TIM1_CH1,
                              .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                              .pin       = IO::Timer::pinT1Ch1,
                              .port      = IO::Timer::portT1Ch1,
                              .pull      = Chimera::GPIO::Pull::PULL_DN,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true },
                            { // Inverter High Side Phase B
                              .alternate = Chimera::GPIO::Alternate::TIM1_CH2,
                              .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                              .pin       = IO::Timer::pinT1Ch2,
                              .port      = IO::Timer::portT1Ch2,
                              .pull      = Chimera::GPIO::Pull::PULL_DN,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true },
                            { // Inverter High Side Phase C
                              .alternate = Chimera::GPIO::Alternate::TIM1_CH3,
                              .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                              .pin       = IO::Timer::pinT1Ch3,
                              .port      = IO::Timer::portT1Ch3,
                              .pull      = Chimera::GPIO::Pull::PULL_DN,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true },
                            { // Inverter Low Side Phase A
                              .alternate = Chimera::GPIO::Alternate::TIM1_CH1N,
                              .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                              .pin       = IO::Timer::pinT1Ch1N,
                              .port      = IO::Timer::portT1Ch1N,
                              .pull      = Chimera::GPIO::Pull::PULL_DN,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true },
                            { // Inverter Low Side Phase B
                              .alternate = Chimera::GPIO::Alternate::TIM1_CH2N,
                              .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                              .pin       = IO::Timer::pinT1Ch2N,
                              .port      = IO::Timer::portT1Ch2N,
                              .pull      = Chimera::GPIO::Pull::PULL_DN,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true },
                            { // Inverter Low Side Phase C
                              .alternate = Chimera::GPIO::Alternate::TIM1_CH3N,
                              .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                              .pin       = IO::Timer::pinT1Ch3N,
                              .port      = IO::Timer::portT1Ch3N,
                              .pull      = Chimera::GPIO::Pull::PULL_DN,
                              .state     = Chimera::GPIO::State::LOW,
                              .threaded  = true,
                              .validity  = true }, };

    /*-------------------------------------------------------------------------
    Configure the GPIO
    -------------------------------------------------------------------------*/
    for ( size_t pinIdx = 0; pinIdx < ARRAY_COUNT( cfg ); pinIdx++ )
    {
      auto pin = Chimera::GPIO::getDriver( cfg[ pinIdx ].port, cfg[ pinIdx ].pin );
      RT_HARD_ASSERT( pin != nullptr );
      RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg[ pinIdx ] ) );
    }

    /*-------------------------------------------------------------------------
    Configure the Heartbeat LED PWM timer
    -------------------------------------------------------------------------*/
    Chimera::Timer::PWM::DriverConfig pwm_cfg;
    pwm_cfg.clear();
    pwm_cfg.polarity            = Chimera::Timer::PWM::Polarity::ACTIVE_HIGH;
    pwm_cfg.safeIOLevel         = Chimera::GPIO::State::HIGH;
    pwm_cfg.coreCfg.instance    = Chimera::Timer::Instance::TIMER16;
    pwm_cfg.coreCfg.baseFreq    = 1'000'000.0f;
    pwm_cfg.coreCfg.clockSource = Chimera::Clock::Bus::SYSCLK;
    pwm_cfg.channel             = Chimera::Timer::Channel::CHANNEL_1;
    pwm_cfg.output              = Chimera::Timer::Output::OUTPUT_1P;
    pwm_cfg.dutyCycle           = 50.0f;
    pwm_cfg.frequency           = 10000.0f;

    RT_HARD_ASSERT( Chimera::Status::OK == PWMDriver.init( pwm_cfg ) );
    PWMDriver.enableOutput();

    /*-------------------------------------------------------------------------
    Configure the ADC sample timer
    TODO BMB: Replaced by the FOC controller?
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = 1000.0f;
    trig_cfg.coreConfig.instance    = Chimera::Timer::Instance::TIMER15;
    trig_cfg.coreConfig.baseFreq    = 1'000'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == ADCTrigger.init( trig_cfg ) );
    ADCTrigger.enable();
  }

}  // namespace Orbit::TIMER
