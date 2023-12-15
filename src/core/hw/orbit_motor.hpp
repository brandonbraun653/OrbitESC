/******************************************************************************
 *  File Name:
 *    orbit_motor.hpp
 *
 *  Description:
 *    Common motor hardware controller interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_MOTOR_HPP
#define ORBIT_ESC_MOTOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::Motor
{

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief Base frequency of the motor drive and sense timers.
   *
   * The same frequency is used for both timers to that they share the
   * same timebase and event period. This makes it easier to synchronize.
   */
  static constexpr uint32_t TIMER_BASE_FREQ = 60'000'000;

}  // namespace Orbit::Motor

#endif  /* !ORBIT_ESC_MOTOR_HPP */
