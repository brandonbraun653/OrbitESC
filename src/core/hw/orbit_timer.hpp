/******************************************************************************
 *  File Name:
 *    orbit_timer.hpp
 *
 *  Description:
 *    Orbit ESC TIMER Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TIMER_HPP
#define ORBIT_ESC_TIMER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>


namespace Orbit::TIMER
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern Chimera::Timer::PWM::Driver PWMDriver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the TIMER driver subsystem
   */
  void powerUp();

}  // namespace Orbit::TIMER

#endif  /* !ORBIT_ESC_TIMER_HPP */
