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


  ClarkeSpace clarke_transform( const float a, const float b )
  {
    return { .alpha = a, .beta = ( a * ONE_OVER_SQRT3 ) + ( b * TWO_OVER_SQRT3 ) };
  }


  ParkSpace park_transform( const ClarkeSpace &clarke, const float angle_est )
  {
    /*-------------------------------------------------------------------------
    Cache the sine/cosine of the angle estimate
    -------------------------------------------------------------------------*/
    float sin, cos;
    fast_sin_cos( angle_est, &sin, &cos );

    /*-------------------------------------------------------------------------
    Park transform
    -------------------------------------------------------------------------*/
    /* clang-format off */
    return { .d = (  clarke.alpha * cos ) + ( clarke.beta * sin ),
             .q = ( -clarke.alpha * sin ) + ( clarke.beta * cos ) };
    /* clang-format on */
  }

}    // namespace Orbit::Control::Math
