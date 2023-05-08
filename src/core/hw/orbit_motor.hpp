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


namespace Orbit::Motor
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum ADCChannel
  {
    ADC_CH_MOTOR_PHASE_A_CURRENT,
    ADC_CH_MOTOR_PHASE_B_CURRENT,
    ADC_CH_MOTOR_PHASE_C_CURRENT,
    ADC_CH_MOTOR_SUPPLY_VOLTAGE,

    ADC_CH_NUM_OPTIONS,
    ADC_CH_INVALID
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the motor driver subsystem
   */
  void powerUp();

  /**
   * @brief Immediately stops the power stage and disables the motor driver
   * @note  This is a hard stop and will not attempt to decelerate the motor
   *
   * Sends a break input to the timer generating the PWM signal.
   */
  void emergencyStop();

}  // namespace Orbit::Motor

#endif  /* !ORBIT_MOTOR_HPP */
