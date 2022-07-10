/******************************************************************************
 *  File Name:
 *    foc_driver.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FOC_CONTROL_HPP
#define ORBIT_ESC_FOC_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/common>
#include <Chimera/adc>
#include <Chimera/timer>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class ADCChannel
  {
    MOTOR_PHASE_A_CURRENT,
    MOTOR_PHASE_B_CURRENT,
    MOTOR_SUPPLY_VOLTAGE,

    NUM_OPTIONS,
    INVALID
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct FOCConfig
  {
    Chimera::ADC::Peripheral adcSource;
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Brush-less dc motor control library implemented with FOC
   *
   */
  class FOC
  {
  public:
    FOC();
    ~FOC();

    int initialize( const FOCConfig &cfg );

    int setSpeedRef( const float ref );

  protected:
    void dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr );
    void timer_isr_speed_controller();

  private:
    struct InternalState
    {
      uint16_t adc_samples[ EnumValue( ADCChannel::NUM_OPTIONS ) ];
    };

    Chimera::ADC::Driver_rPtr mADCDriver;
    Chimera::Timer::TriPhasePWM::Driver mTimerDriver;

    InternalState mPrvState;
  };
}  // namespace Orbit::FOC

#endif  /* !ORBIT_ESC_FOC_CONTROL_HPP */
