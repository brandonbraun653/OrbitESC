/******************************************************************************
 *  File Name:
 *    foc_math.hpp
 *
 *  Description:
 *    Field Oriented Control Math Utilities
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FOC_MATH_HPP
#define ORBIT_ESC_FOC_MATH_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Computes the sine and cosine of an input angle
   * @see https://github.com/vedderb/bldc/blob/master/util/utils_math.c
   *
   * @param angle   Angle to compute the sine and cosine of
   * @param sin     Output pointer to store the sine of the angle
   * @param cos     Output pointer to store the cosine of the angle
   */
  void fast_sin_cos( const float angle, float *const sin, float *const cos );

}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_FOC_MATH_HPP */
