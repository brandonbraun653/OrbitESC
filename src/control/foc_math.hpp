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
#include <cmath>
#include <cstdint>

namespace Orbit::Control::Math
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float ONE_OVER_SQRT3 = 0.57735026918962576451f;
  static constexpr float TWO_OVER_SQRT3 = 1.15470053837925152902f;
  static constexpr float M_PI_F         = static_cast<float>( M_PI );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Container for numbers resulting from a Clarke transform
   */
  struct ClarkeSpace
  {
    float alpha;
    float beta;
  };

  /**
   * @brief Container for numbers resulting from a Park transform
   */
  struct ParkSpace
  {
    float d;
    float q;
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Checks for NaN truthiness
   *
   * @param x     Number to check
   * @return bool True if NaN, false otherwise
   */
  static constexpr bool is_nan( const float x )
  {
    return x != x;
  }

  /**
   * @brief Checks for infinity truthiness
   *
   * @param x     Number to check
   * @return bool True if infinity, false otherwise
   */
  static constexpr bool is_inf( const float x )
  {
    return ( x == 1.0f / 0.0f ) || ( x == -1.0f / 0.0f );
  }

  /**
   * @brief Checks for NaN and clears to zero if true
   *
   * @param x   Number to check/act on
   */
  static inline void clear_if_nan( float &x )
  {
    if ( is_nan( x ) )
    {
      x = 0.0f;
    }
  }

  /**
   * @brief Computes the sine and cosine of an input angle
   * @see https://github.com/vedderb/bldc/blob/master/util/utils_math.c
   *
   * @param angle   Angle to compute the sine and cosine of
   * @param sin     Output pointer to store the sine of the angle
   * @param cos     Output pointer to store the cosine of the angle
   */
  void fast_sin_cos( float angle, float *const sin, float *const cos );

  /**
   * @brief Computes atan2 quickly, accurately, and with output normalization
   * @see http://dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization/
   *
   * @param y   Numerator input
   * @param x   Denominator input
   * @return float
   */
  float fast_atan2_with_norm( const float y, const float x );

  /**
   * @brief Computes the Clarke transform, disregarding the homopolar component
   * @see https://www.ti.com/lit/an/bpra048/bpra048.pdf
   *
   * @param a   Phase A motor current
   * @param b   Phase B motor current
   * @return ClarkeSpace
   */
  ClarkeSpace clarke_transform( const float a, const float b );

  /**
   * @brief Computes the Park transform of the Clarke space input
   * @see https://www.ti.com/lit/an/bpra048/bpra048.pdf
   *
   * @param clarke  Clarke space data to transform
   * @param angle   Estimated angle of the motor
   * @return ParkSpace
   */
  ParkSpace park_transform( const ClarkeSpace &clarke, const float angle_est );

}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_FOC_MATH_HPP */
