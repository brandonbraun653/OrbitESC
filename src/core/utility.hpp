/******************************************************************************
 *  File Name:
 *    utility.hpp
 *
 *  Description:
 *    Utility functions to support the project
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_UTILITY_HPP
#define ORBIT_UTILITY_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace Orbit::Utility
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Calculates the number of electrical commutation events needed to achieve 1 revolution
   *
   * @param poles   How many poles does the motor have
   * @return size_t
   */
  constexpr size_t eComEventsPerRev( const size_t poles )
  {
    return poles * 6u;
  }

  /**
   * @brief Calculates PWM cycles per commutation event to achieve a particular mechanical revolution rate
   *
   * @param freqSwitch  Power stage drive frequency
   * @param poles       Motor poles
   * @param rpm         Target RPM
   * @return size_t     How many PWM cycles till the next commutation event should occur
   */
  size_t comCycleCount( const float freqSwitch, const size_t poles, const size_t rpm );

}  // namespace Orbit::Utility

#endif  /* !ORBIT_UTILITY_HPP */
