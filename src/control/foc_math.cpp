/******************************************************************************
 *  File Name:
 *    foc_math.cpp
 *
 *  Description:
 *    Field Oriented Control Math Utilities
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
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


  /*---------------------------------------------------------------------------
  Trapezoidal Integrator
  ---------------------------------------------------------------------------*/
  TrapInt::TrapInt() :
      K( 1.0f ), Min( 0.0f ), Max( 0.0f ), Y( 0.0f ), ULast( 0.0f )
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
