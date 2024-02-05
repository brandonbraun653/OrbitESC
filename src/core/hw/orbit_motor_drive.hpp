/******************************************************************************
 *  File Name:
 *    orbit_motor_drive.hpp
 *
 *  Description:
 *    Motor controller HW interface
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_MOTOR_DRIVE_HPP
#define ORBIT_MOTOR_DRIVE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/timer>
#include <cstdint>
#include <src/core/hw/orbit_motor.hpp>


namespace Orbit::Motor::Drive
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

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
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Retrieves the driver instance
   * @return Chimera::Timer::Inverter::Driver*
   */
  Chimera::Timer::Inverter::Driver *getDriver();

  /**
   * @brief Powers up the motor driver output subsystem.
   * @return void
   */
  void initialize();

  /**
   * @brief Reset the motor driver to a known state.
   * @return void
   */
  void reset();

  /**
   * @brief Immediately stops the power stage and disables the motor driver via HW signal.
   * @note Only use in panic situations. Use disableOutput() for controlled stops.
   *
   * Sends a break input to the timer generating the PWM signal, effectively cutting off
   * the power stage from the motor. This is a hard stop and may result in damage to the
   * motor or power stage.
   *
   * @return void
   */
  void emergencyStop();

  /**
   * @brief Enable the power stage drive signals
   * @return void
   */
  void enableOutput();

  /**
   * @brief Disable the power stage drive signals.
   *
   * This is a low priority (aka slow) method of disabling the motor drive and is
   * intended for controlled stops only. Use emergencyStop() for panic situations
   * where the power stage needs to shut off immediately.
   */
  void disableOutput();

  /**
   * @brief Updates drive outputs using space vector modulation
   *
   * @param alpha   Output of inverse park transform
   * @param beta    Output of inverse park transform
   * @param theta   Desired electrical angle of the resulting drive vector in radians
   * @return Chimera::Status_t
   */
  void svmUpdate( const float &alpha, const float &beta, const float &theta );

  /**
   * @brief Retrieves the number of timer ticks each high side switch is on for.
   *
   * These values can be used to determine which phases have the most time
   * available for ADC sampling.
   *
   * @param tOnA  Phase A high side on time in timer ticks
   * @param tOnB  Phase B high side on time in timer ticks
   * @param tOnC  Phase C high side on time in timer ticks
   * @return void
   */
  void svmOnTicks( uint32_t &tOnA, uint32_t &tOnB, uint32_t &tOnC );

}    // namespace Orbit::Motor

#endif /* !ORBIT_MOTOR_DRIVE_HPP */
