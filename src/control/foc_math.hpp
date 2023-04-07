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
#include <limits>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/
#define US_TO_SEC( x ) ( static_cast<float>( x ) / 1e6f )
#define RAD_TO_RPM( rad ) ( static_cast<float>( rad ) * static_cast<float>( 60.0 / ( 2.0 * M_PI ) ) )
#define RPM_TO_RAD( rpm ) ( ( static_cast<float>( rpm ) / 60.0f ) * static_cast<float>( 2.0 * M_PI ) )

namespace Orbit::Control::Math
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float ONE_OVER_SQRT3 = 0.57735026918962576451f;
  static constexpr float TWO_OVER_SQRT3 = 1.15470053837925152902f;
  static constexpr float SQRT3_OVER_2   = 0.86602540378443864676f;
  static constexpr float M_PI_F         = static_cast<float>( M_PI );
  static constexpr float M_2PI_F        = 2.0f * M_PI_F;

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
   * @brief Checks for "almost-equality" between floating point values
   *
   * @param x         First value to check
   * @param y         Second value to check
   * @param epsilon   Precision in the comparison
   * @return bool     True if nearly equal, false otherwise
   */
  static constexpr bool is_nearly_equal( const float x, const float y, const float epsilon = 1e-5f )
  {
    return fabsf( x - y ) <= ( epsilon * fabsf( x ) );
  }

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
   * @brief Computes the sine of an input angle
   * @note Sine only version of fast_sin_cos()
   *
   * @param angle   Angle to compute the sine of
   * @param sin     Output pointer to store the sine of the angle
   */
  void fast_sin( float angle, float *const sin );

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
   * @param a       Phase A motor current
   * @param b       Phase B motor current
   * @param alpha   Output reference to store the alpha component
   * @param beta    Output reference to store the beta component
   * @return void
   */
  void clarke_transform( const float a, const float b, float &alpha, float &beta );

  /**
   * @brief Computes the Park transform of the Clarke space input
   * @see https://www.ti.com/lit/an/bpra048/bpra048.pdf
   *
   * @param alpha   Alpha component of the Clarke space data
   * @param beta    Beta component of the Clarke space data
   * @param theta   Estimated angle of the motor
   * @param q       Output reference to store the q component
   * @param d       Output reference to store the d component
   * @return void
   */
  void park_transform( const float alpha, const float beta, const float theta, float &q, float &d );

  /**
   * @brief Computes the inverse Park transform of the Park space input
   * @see https://www.ti.com/lit/an/bpra048/bpra048.pdf
   *
   * @param park        Park space data to transform
   * @param angle_est   Estimated angle of the motor
   * @return ClarkeSpace
   */
  ClarkeSpace inverse_park_transform( const ParkSpace &park, const float angle_est );

  /**
   * @brief Computes the inverse Clarke transform of the Clarke space input
   * @see https://www.ti.com/lit/an/bpra048/bpra048.pdf
   *
   * @param clarke  Clarke space data to transform
   * @param a       Output pointer to store the phase A data
   * @param b      Output pointer to store the phase B data
   * @param c       Output pointer to store the phase C data
   * @return void
   */
  void inverse_clarke_transform( const ClarkeSpace &clark, float *const a, float *const b, float *const c );

  /**
   * @brief Clips an input value to be bounded between min/max
   *
   * @param v       Value being clipped
   * @param min     Minimum limit
   * @param max     Maximum limit
   * @return float
   */
  float clamp( const float v, const float min, const float max );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Discrete time trapezoidal integrator
   * @see https://www.mathworks.com/help/simulink/slref/discretetimeintegrator.html
   */
  class TrapInt
  {
  public:
    float K;  /**< Proportionality constant to scale integration by*/

    TrapInt();
    ~TrapInt();

    /**
     * @brief Reset the integrator
     *
     * @param ic    Initial condition
     * @param min   Minimum value saturation limit
     * @param max   Maximum value saturation limit
     */
    void reset( const float ic = 0.0f, const float min = std::numeric_limits<float>::min(),
                const float max = std::numeric_limits<float>::max() );

    /**
     * @brief Steps the integration forward by dt
     *
     * @param u   Input value for this time step
     * @param dt  Time to integrate over
     * @return float
     */
    float step( const float u, const float dt );

    /**
     * @brief Get the current value of the integration
     * @return float
     */
    float value() const;

  private:
    float Min;
    float Max;
    float Y;
    float ULast;
  };

}    // namespace Orbit::Control::Math

#endif /* !ORBIT_ESC_FOC_MATH_HPP */
