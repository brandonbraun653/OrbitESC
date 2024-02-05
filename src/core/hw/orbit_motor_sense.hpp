/******************************************************************************
 *  File Name:
 *    orbit_motor_sense.hpp
 *
 *  Description:
 *    Motor controller HW sensor interface
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_MOTOR_SENSEHPP
#define ORBIT_MOTOR_SENSEHPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/timer>
#include <cstdint>
#include <src/core/hw/orbit_motor.hpp>


namespace Orbit::Motor::Sense
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using SenseCallback = void ( * )( void );

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Ordering of the ADC channels in the sample buffer.
   *
   * The sampling order of the ADC is used in several places for both
   * configuring and interpreting data, so it's convenient to define it once.
   */
  enum ChannelSequence : uint8_t
  {
    CHANNEL_PHASE_A_CURRENT,
    CHANNEL_PHASE_B_CURRENT,
    CHANNEL_PHASE_C_CURRENT,
    CHANNEL_PHASE_A_VOLTAGE,
    CHANNEL_PHASE_B_VOLTAGE,
    CHANNEL_PHASE_C_VOLTAGE,

    CHANNEL_COUNT
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Data structure for holding the results of a motor sense measurement
   * @note All measurements are in SI units (volts, amps, seconds).
   */
  struct SenseData
  {
    float channel[ CHANNEL_COUNT ]; /**< Voltage/current measurements */
    float timestamp;                /**< System time of the measurement */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Gets the timer used for triggering ADC conversions
   * @return Chimera::Timer::Trigger::Slave*
   */
  Chimera::Timer::Trigger::Slave *getTimer();

  /**
   * @brief Powers up the motor driver sensor subsystem.
   * @return void
   */
  void initialize();

  /**
   * @brief Reset the motor sense subsystem
   * @return void
   */
  void reset();

  /**
   * @brief Sets the callback to invoke when the ADC completes a conversion
   *
   * @param callback  Callback to invoke
   * @return void
   */
  void onComplete( SenseCallback callback );

  /**
   * @brief Gets the latest motor sense data
   * @return Const reference to the data
   */
  volatile const SenseData &getSenseData();

  /**
   * @brief Calibrate the motor current/voltage sense inputs
   * @return void
   */
  void calibrate();

}    // namespace Orbit::Motor::Sense

#endif /* !ORBIT_MOTOR_SENSEHPP */
