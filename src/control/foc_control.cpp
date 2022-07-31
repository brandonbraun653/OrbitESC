/******************************************************************************
 *  File Name:
 *    foc_control.cpp
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
#include <src/control/foc_math.hpp>

/*-----------------------------------------------------------------------------
Local Literals
-----------------------------------------------------------------------------*/
#define M_PI_F ( static_cast<float>( M_PI ) )

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void fast_sin_cos( float angle, float *const sin, float *const cos )
  {
    /*-------------------------------------------------------------------------
    Wrap the input angle to -PI to PI
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
    Compute sine
    -------------------------------------------------------------------------*/
    if ( angle < 0.0f )
    {
      *sin = 1.27323954f * angle + 0.405284735f * angle * angle;

      if ( *sin < 0.0f )
      {
        *sin = 0.225f * ( *sin * -*sin - *sin ) + *sin;
      }
      else
      {
        *sin = 0.225f * ( *sin * *sin - *sin ) + *sin;
      }
    }
    else
    {
      *sin = 1.27323954f * angle - 0.405284735f * angle * angle;

      if ( *sin < 0.0f )
      {
        *sin = 0.225f * ( *sin * -*sin - *sin ) + *sin;
      }
      else
      {
        *sin = 0.225f * ( *sin * *sin - *sin ) + *sin;
      }
    }

    /*-------------------------------------------------------------------------
    Compute cosine: sin(x + PI/2) = cos(x)
    -------------------------------------------------------------------------*/
    angle += 0.5f * M_PI_F;
    if ( angle > M_PI_F )
    {
      angle -= 2.0f * M_PI_F;
    }

    if ( angle < 0.0f )
    {
      *cos = 1.27323954f * angle + 0.405284735f * angle * angle;

      if ( *cos < 0.0f )
      {
        *cos = 0.225f * ( *cos * -*cos - *cos ) + *cos;
      }
      else
      {
        *cos = 0.225f * ( *cos * *cos - *cos ) + *cos;
      }
    }
    else
    {
      *cos = 1.27323954f * angle - 0.405284735f * angle * angle;

      if ( *cos < 0.0f )
      {
        *cos = 0.225f * ( *cos * -*cos - *cos ) + *cos;
      }
      else
      {
        *cos = 0.225f * ( *cos * *cos - *cos ) + *cos;
      }
    }
  }
}    // namespace Orbit::Control
