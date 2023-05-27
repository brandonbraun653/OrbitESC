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
#define RAD_TO_DEG( rad ) ( static_cast<float>( rad ) * static_cast<float>( 180.0 / M_PI ) )
#define DEG_TO_RAD( deg ) ( static_cast<float>( deg ) * static_cast<float>( M_PI / 180.0 ) )

#define SQ( x ) ( ( x ) * ( x ) )
#define NORM2_f( x, y ) ( sqrtf( SQ( x ) + SQ( y ) ) )

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
   * @brief Truncates a floating point value to a maximum value
   *
   * @param x   Value to truncate
   * @param max Maximum value to truncate to
   */
  static inline void truncate_fabs( float &x, const float max )
  {
    if( x > max )
    {
      x = max;
    }
    else if( x < -max )
    {
      x = -max;
    }
  }

  /**
   * @brief Limits the magnitude of the vector to a maximum value
   *
   * @param x       X component of the vector
   * @param y       Y component of the vector
   * @param max     Maximum magnitude of the vector
   * @return true   Vector was saturated
   * @return false  Vector was not saturated
   */
  static inline bool saturate_vector_2d( float &x, float &y, float max )
  {
    bool  retval = false;
    float mag    = NORM2_f( x, y );
    max          = fabsf( max );

    if ( mag < 1e-10 )
    {
      mag = 1e-10;
    }

    if ( mag > max )
    {
      const float f = max / mag;
      x *= f;
      y *= f;
      retval = true;
    }

    return retval;
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
   * @param q       Q component
   * @param d       D component
   * @param theta   Estimated angle of the motor
   * @param a       Output reference to store the alpha component
   * @param b       Output reference to store the beta component
   * @return void
   */
  void inverse_park_transform( const float q, const float d, const float theta, float &a, float &b );

  /**
   * @brief Computes the inverse Clarke transform of the Clarke space input
   * @see https://www.ti.com/lit/an/bpra048/bpra048.pdf
   *
   * @param a       Alpha component
   * @param b       Beta component
   * @param v1      Output reference to store the phase A voltage component
   * @param v2      Output reference to store the phase B voltage component
   * @param v3      Output reference to store the phase C voltage component
   * @return void
   */
  void inverse_clarke_transform( const float a, const float b, float &v1, float &v2, float &v3 );

  /**
   * @brief Clips an input value to be bounded between min/max
   *
   * @param v       Value being clipped
   * @param min     Minimum limit
   * @param max     Maximum limit
   * @return float
   */
  float clamp( const float v, const float min, const float max );

  /**
   * @brief Performs Space Vector Modulation on the input alpha/beta vector
   * @see https://github.com/vedderb/bldc
   *
   * @param alpha               Alpha component of the vector
   * @param beta                Beta component of the vector
   * @param timer_pwm_arr       Timer PWM ARR value, corresponding to full duty cycle
   * @param timer_pwm_ccr1      Output reference to store the timer PWM CCR1 value
   * @param timer_pwm_ccr2      Output reference to store the timer PWM CCR2 value
   * @param timer_pwm_ccr3      Output reference to store the timer PWM CCR3 value
   * @param commutation_sector  Output reference to store the new commutation sector
   */
  void space_vector_modulation( const float alpha, const float beta, const uint32_t timer_pwm_arr, uint32_t &timer_pwm_ccr1,
                                uint32_t &timer_pwm_ccr2, uint32_t &timer_pwm_ccr3, uint32_t &commutation_sector );

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
