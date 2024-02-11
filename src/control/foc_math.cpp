/******************************************************************************
 *  File Name:
 *    foc_math.cpp
 *
 *  Description:
 *    Field Oriented Control Math Utilities
 *
 *  2022-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cmath>
#include <Chimera/assert>
#include <src/control/foc_math.hpp>

namespace Orbit::Control::Math
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void fast_sin_cos( float angle, float *const sin, float *const cos )
  {
    RT_DBG_ASSERT( ( sin != nullptr ) && ( cos != nullptr ) );

    /*-------------------------------------------------------------------------
    Wrap the angle from -PI to PI
    -------------------------------------------------------------------------*/
    while ( angle < -M_PI_F )
    {
      angle += 2.0f * M_PI_F;
    }

    while ( angle > M_PI_F )
    {
      angle -= 2.0f * M_PI_F;
    }

    /*-------------------------------------------------------------------------
    Compute Sine
    -------------------------------------------------------------------------*/
    if ( angle < 0.0f )
    {
      *sin = 1.27323954f * angle + 0.405284735f * angle * angle;
    }
    else
    {
      *sin = 1.27323954f * angle - 0.405284735f * angle * angle;
    }

    /*-------------------------------------------------------------------------
    Compute Cosine: sin(x + PI/2) = cos(x)
    -------------------------------------------------------------------------*/
    angle += 0.5f * M_PI_F;

    if ( angle > M_PI_F )
    {
      angle -= 2.0f * M_PI_F;
    }

    if ( angle < 0.0f )
    {
      *cos = 1.27323954f * angle + 0.405284735f * angle * angle;
    }
    else
    {
      *cos = 1.27323954f * angle - 0.405284735f * angle * angle;
    }
  }


  void fast_sin( float angle, float *const sin )
  {
    RT_DBG_ASSERT( ( sin != nullptr ) && ( cos != nullptr ) );

    /*-------------------------------------------------------------------------
    Wrap the angle from -PI to PI
    -------------------------------------------------------------------------*/
    while ( angle < -M_PI_F )
    {
      angle += 2.0f * M_PI_F;
    }

    while ( angle > M_PI_F )
    {
      angle -= 2.0f * M_PI_F;
    }

    /*-------------------------------------------------------------------------
    Compute Sine
    -------------------------------------------------------------------------*/
    if ( angle < 0.0f )
    {
      *sin = 1.27323954f * angle + 0.405284735f * angle * angle;
    }
    else
    {
      *sin = 1.27323954f * angle - 0.405284735f * angle * angle;
    }
  }


  float fast_atan2_with_norm( const float y, const float x )
  {
    const float abs_y = fabsf( y ) + 1e-20f;    // kludge to prevent 0/0 condition
    float       angle;

    if ( x >= 0.0f )
    {
      const float r   = ( x - abs_y ) / ( x + abs_y );
      const float rsq = r * r;
      angle           = ( ( 0.1963f * rsq ) - 0.9817f ) * r + ( M_PI_F / 4.0f );
    }
    else
    {
      const float r   = ( x + abs_y ) / ( abs_y - x );
      const float rsq = r * r;
      angle           = ( ( 0.1963f * rsq ) - 0.9817f ) * r + ( 3.0f * M_PI_F / 4.0f );
    }

    clear_if_nan( angle );

    if ( y < 0.0f )
    {
      return ( -angle );
    }
    else
    {
      return ( angle );
    }
  }


  void clarke_transform( const float a, const float b, float &alpha, float &beta )
  {
    alpha = a;
    beta  = ( a * ONE_OVER_SQRT3 ) + ( b * TWO_OVER_SQRT3 );
  }


  void park_transform( const float alpha, const float beta, const float theta, float &q, float &d )
  {
    /*-------------------------------------------------------------------------
    Cache the sine/cosine of the angle estimate
    -------------------------------------------------------------------------*/
    float sin, cos;
    fast_sin_cos( theta, &sin, &cos );

    /*-------------------------------------------------------------------------
    Park transform
    -------------------------------------------------------------------------*/
    d = ( alpha * cos ) + ( beta * sin );
    q = ( -alpha * sin ) + ( beta * cos );
  }


  void inverse_park_transform( const float q, const float d, const float theta, float &a, float &b )
  {
    /*-------------------------------------------------------------------------
    Cache the sine/cosine of the angle estimate
    -------------------------------------------------------------------------*/
    float sin, cos;
    fast_sin_cos( theta, &sin, &cos );

    /*-------------------------------------------------------------------------
    Inverse Park transform
    -------------------------------------------------------------------------*/
    a = ( d * cos ) - ( q * sin );
    b = ( d * sin ) + ( q * cos );
  }


  void inverse_clarke_transform( const float a, const float b, float &v1, float &v2, float &v3 )
  {
    v1 = a;
    v2 = -0.5f * a + b * SQRT3_OVER_2;
    v3 = -0.5f * a - b * SQRT3_OVER_2;
  }


  float clamp( const float v, const float min, const float max )
  {
    return std::max( min, std::min( v, max ) );
  }

  // TODO BMB: Does this need removal? I think I'm already doing this in the HW timer class.
  void space_vector_modulation( const float alpha, const float beta, const uint32_t timer_pwm_arr, uint32_t &timer_pwm_ccr1,
                                uint32_t &timer_pwm_ccr2, uint32_t &timer_pwm_ccr3, uint32_t &commutation_sector )
  {
    /*-------------------------------------------------------------------------
    Compute the current sector
    -------------------------------------------------------------------------*/
    uint32_t sector;
    if ( beta >= 0.0f )
    {
      if ( alpha >= 0.0f )
      {
        sector = ONE_OVER_SQRT3 * beta > alpha ? 2 : 1;
      }
      else
      {
        sector = -ONE_OVER_SQRT3 * beta > alpha ? 3 : 2;
      }
    }
    else
    {
      if ( alpha >= 0.0f )
      {
        sector = -ONE_OVER_SQRT3 * beta > alpha ? 5 : 6;
      }
      else
      {
        sector = ONE_OVER_SQRT3 * beta > alpha ? 4 : 5;
      }
    }

    /*-------------------------------------------------------------------------
    Compute the duty cycles for each PWM channel
    -------------------------------------------------------------------------*/
    uint32_t tA, tB, tC;
    switch ( sector )
    {
      // sector 1-2
      case 1: {
        // Vector on-times
        uint32_t t1 = ( alpha - ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;
        uint32_t t2 = ( TWO_OVER_SQRT3 * beta ) * timer_pwm_arr;

        // PWM timings
        tA = ( timer_pwm_arr + t1 + t2 ) / 2;
        tB = tA - t1;
        tC = tB - t2;

        break;
      }

      // sector 2-3
      case 2: {
        // Vector on-times
        uint32_t t2 = ( alpha + ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;
        uint32_t t3 = ( -alpha + ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;

        // PWM timings
        tB = ( timer_pwm_arr + t2 + t3 ) / 2;
        tA = tB - t3;
        tC = tA - t2;

        break;
      }

      // sector 3-4
      case 3: {
        // Vector on-times
        uint32_t t3 = ( TWO_OVER_SQRT3 * beta ) * timer_pwm_arr;
        uint32_t t4 = ( -alpha - ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;

        // PWM timings
        tB = ( timer_pwm_arr + t3 + t4 ) / 2;
        tC = tB - t3;
        tA = tC - t4;

        break;
      }

      // sector 4-5
      case 4: {
        // Vector on-times
        uint32_t t4 = ( -alpha + ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;
        uint32_t t5 = ( -TWO_OVER_SQRT3 * beta ) * timer_pwm_arr;

        // PWM timings
        tC = ( timer_pwm_arr + t4 + t5 ) / 2;
        tB = tC - t5;
        tA = tB - t4;

        break;
      }

      // sector 5-6
      case 5: {
        // Vector on-times
        uint32_t t5 = ( -alpha - ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;
        uint32_t t6 = ( alpha - ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;

        // PWM timings
        tC = ( timer_pwm_arr + t5 + t6 ) / 2;
        tA = tC - t5;
        tB = tA - t6;

        break;
      }

      // sector 6-1
      case 6: {
        // Vector on-times
        uint32_t t6 = ( -TWO_OVER_SQRT3 * beta ) * timer_pwm_arr;
        uint32_t t1 = ( alpha + ONE_OVER_SQRT3 * beta ) * timer_pwm_arr;

        // PWM timings
        tA = ( timer_pwm_arr + t6 + t1 ) / 2;
        tC = tA - t1;
        tB = tC - t6;

        break;
      }
    }

    /*-------------------------------------------------------------------------
    Assign output variables
    -------------------------------------------------------------------------*/
    timer_pwm_ccr1     = tA;
    timer_pwm_ccr2     = tB;
    timer_pwm_ccr3     = tC;
    commutation_sector = sector;
  }

  /*---------------------------------------------------------------------------
  Trapezoidal Integrator
  ---------------------------------------------------------------------------*/
  TrapInt::TrapInt() : K( 1.0f ), Min( 0.0f ), Max( 0.0f ), Y( 0.0f ), ULast( 0.0f )
  {
  }


  TrapInt::~TrapInt()
  {
  }


  void TrapInt::reset( const float ic, const float min, const float max )
  {
    Min   = min;
    Max   = max;
    Y     = clamp( ic, Min, Max );
    ULast = 0.0f;
  }


  float TrapInt::step( const float u, const float dt )
  {
    Y = Y + ( K * dt * ( u + ULast ) ) / 2.0f;
    Y = clamp( Y, Min, Max );

    ULast = u;
    return Y;
  }


  float TrapInt::value() const
  {
    return Y;
  }

}    // namespace Orbit::Control::Math
