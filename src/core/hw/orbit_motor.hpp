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
   * @brief Flushes any queued data from the motor driver subsystem
   *
   * This will transmit messages on the debug port to the host PC for the purpose
   * of visualizing the motor driver data.
   */
  void flushDataQueue();

}  // namespace Orbit::Motor

#endif  /* !ORBIT_MOTOR_HPP */
