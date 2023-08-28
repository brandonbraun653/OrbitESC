/******************************************************************************
 *  File Name:
 *    orbit_motor.hpp
 *
 *  Description:
 *    Motor controller HW interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_MOTOR_HPP
#define ORBIT_MOTOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/function>


namespace Orbit::Motor
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using SenseCallback = void ( * )( void );

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief Base frequency of the motor drive and sense timers.
   *
   * The same frequency is used for both timers to that they share the
   * same timebase and event period. This makes it easier to synchronize.
   */
  static constexpr uint32_t TIMER_BASE_FREQ = 10'000'000;

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


  enum Rotation : uint8_t
  {
    ROTATION_CW,
    ROTATION_CCW
  };


  enum DriveSector : uint8_t
  {
    SECTOR_1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6,

    SECTOR_COUNT
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
    float channel[ CHANNEL_COUNT ];  /**< Voltage/current measurements */
    float timestamp;                 /**< System time of the measurement */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Powers up the motor driver sensor subsystem.
   * @return void
   */
  void powerUpSense();

  /**
   * @brief Powers up the motor driver output subsystem.
   * @return void
   */
  void powerUpDrive();

  /**
   * @brief Immediately stops the power stage and disables the motor driver via HW signal.
   * @note Only use in panic situations. Use disableDriveOutput() for controlled stops.
   *
   * Sends a break input to the timer generating the PWM signal, effectively cutting off
   * the power stage from the motor. This is a hard stop and may result in damage to the
   * motor or power stage.
   *
   * @return void
   */
  inline void emergencyStop();

  /**
   * @brief Enable the power stage drive signals
   * @return void
   */
  void enableDriveOutput();

  /**
   * @brief Disable the power stage drive signals.
   *
   * This is a low priority (aka slow) method of disabling the motor drive and is
   * intended for controlled stops only. Use emergencyStop() for panic situations
   * where the power stage needs to shut off immediately.
   */
  void disableDriveOutput();

  /**
   * @brief Updates the timing of when the drive signals should be fired.
   *
   * Sets the power stage timer capture compare registers to control output PWM width for
   * each phase. It's implied that these values are ready for immediate use by the timer
   * peripheral on the next cycle.
   *
   * @param a Offset to apply to phase A
   * @param b Offset to apply to phase B
   * @param c Offset to apply to phase C
   * @return void
   */
  void setDrivePhaseWidth( const uint32_t a, const uint32_t b, const uint32_t c );

  /**
   * @brief Set drive output commutation
   *
   * @param direction Current rotation direction
   * @param sector    New sector to commutate at
   * @return void
   */
  void setDriveCommutation( const Rotation direction, const DriveSector sector );

  /**
   * @brief Sets the callback to invoke when the ADC completes a conversion
   *
   * @param callback  Callback to invoke
   * @return void
   */
  void setSenseCallback( SenseCallback callback );

  /**
   * @brief Gets the latest motor sense data
   * @return Const reference to the data
   */
  volatile const SenseData& getSenseData();

  /**
   * @brief Calibrate the motor current/voltage sense inputs
   * @return void
   */
  void calibrateSenseInputs();

}  // namespace Orbit::Motor

#endif  /* !ORBIT_MOTOR_HPP */
