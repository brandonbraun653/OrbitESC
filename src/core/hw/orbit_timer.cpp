/******************************************************************************
 *  File Name:
 *    orbit_timer.cpp
 *
 *  Description:
 *    OrbitESC TIMER driver
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
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
    configureIOControl();
  }


  void configureIOControl()
  {
    using namespace Chimera::GPIO;

    /*-------------------------------------------------------------------------
    Configuration Table
    -------------------------------------------------------------------------*/
    const PinInit cfg[] = {
      { // Inverter High Side Phase A
        .alternate = Chimera::GPIO::Alternate::TIM1_CH1,
        .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch1,
        .port      = IO::Timer::portT1Ch1,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter High Side Phase B
        .alternate = Chimera::GPIO::Alternate::TIM1_CH2,
        .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch2,
        .port      = IO::Timer::portT1Ch2,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter High Side Phase C
        .alternate = Chimera::GPIO::Alternate::TIM1_CH3,
        .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch3,
        .port      = IO::Timer::portT1Ch3,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter Low Side Phase A
        .alternate = Chimera::GPIO::Alternate::TIM1_CH1N,
        .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch1N,
        .port      = IO::Timer::portT1Ch1N,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter Low Side Phase B
        .alternate = Chimera::GPIO::Alternate::TIM1_CH2N,
        .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch2N,
        .port      = IO::Timer::portT1Ch2N,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter Low Side Phase C
        .alternate = Chimera::GPIO::Alternate::TIM1_CH3N,
        .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch3N,
        .port      = IO::Timer::portT1Ch3N,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
    };

    /*-------------------------------------------------------------------------
    Configure the GPIO
    -------------------------------------------------------------------------*/
    for ( size_t pinIdx = 0; pinIdx < ARRAY_COUNT( cfg ); pinIdx++ )
    {
      auto pin = Chimera::GPIO::getDriver( cfg[ pinIdx ].port, cfg[ pinIdx ].pin );
      RT_HARD_ASSERT( pin != nullptr );
      RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg[ pinIdx ] ) );
    }
  }


  void configureIOTesting()
  {
    using namespace Chimera::GPIO;

    /*-------------------------------------------------------------------------
    Configuration Table
    -------------------------------------------------------------------------*/
    const PinInit cfg[] = {
      { // Inverter High Side Phase A
        .alternate = Chimera::GPIO::Alternate::NONE,
        .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch1,
        .port      = IO::Timer::portT1Ch1,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter High Side Phase B
        .alternate = Chimera::GPIO::Alternate::NONE,
        .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch2,
        .port      = IO::Timer::portT1Ch2,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter High Side Phase C
        .alternate = Chimera::GPIO::Alternate::NONE,
        .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch3,
        .port      = IO::Timer::portT1Ch3,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter Low Side Phase A
        .alternate = Chimera::GPIO::Alternate::NONE,
        .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch1N,
        .port      = IO::Timer::portT1Ch1N,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter Low Side Phase B
        .alternate = Chimera::GPIO::Alternate::NONE,
        .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch2N,
        .port      = IO::Timer::portT1Ch2N,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
      { // Inverter Low Side Phase C
        .alternate = Chimera::GPIO::Alternate::NONE,
        .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
        .pin       = IO::Timer::pinT1Ch3N,
        .port      = IO::Timer::portT1Ch3N,
        .pull      = Chimera::GPIO::Pull::PULL_DN,
        .state     = Chimera::GPIO::State::LOW,
        .threaded  = true,
        .validity  = true },
    };

    /*-------------------------------------------------------------------------
    Configure the GPIO
    -------------------------------------------------------------------------*/
    for ( size_t pinIdx = 0; pinIdx < ARRAY_COUNT( cfg ); pinIdx++ )
    {
      auto pin = Chimera::GPIO::getDriver( cfg[ pinIdx ].port, cfg[ pinIdx ].pin );
      RT_HARD_ASSERT( pin != nullptr );
      RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg[ pinIdx ] ) );
    }
  }
}    // namespace Orbit::TIMER
