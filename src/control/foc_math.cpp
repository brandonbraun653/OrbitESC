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
  // namespace
  // {
  //   constexpr float FAST_SIN_K1 = 1.27323954f;
  //   constexpr float FAST_SIN_K2 = 0.405284735f;
  //   constexpr float FAST_SIN_K3 = 0.225f;

  //   float fast_sin_core( float angle )
  //   {
  //     float result;

  //     if ( angle < 0.0f )
  //     {
  //       result = ( FAST_SIN_K1 * angle ) + ( FAST_SIN_K2 * angle * angle );
  //     }
  //     else
  //     {
  //       result = ( FAST_SIN_K1 * angle ) - ( FAST_SIN_K2 * angle * angle );
  //     }

  //     if ( result < 0.0f )
  //     {
  //       result = ( FAST_SIN_K3 * ( ( result * -result ) - 1.0f ) ) + result;
  //     }
  //     else
  //     {
  //       result = ( FAST_SIN_K3 * ( ( result * result ) - 1.0f ) ) + result;
  //     }

  //     return result;
  //   }
  // }    // namespace

  void fast_sin_cos( float angle, float *const sin, float *const cos )
  {
    RT_DBG_ASSERT( ( sin != nullptr ) && ( cos != nullptr ) );

    /*-------------------------------------------------------------------------
    Wrap the angle from -PI to PI
    -------------------------------------------------------------------------*/
    while( angle < -M_PI_F )
    {
      angle += M_2PI_F;
    }

    while( angle > M_PI_F )
    {
      angle -= M_2PI_F;
    }

    /*-------------------------------------------------------------------------
    Compute Sine/Cosine
    -------------------------------------------------------------------------*/
    // TODO: Replace with the approximations later.
    *sin = std::sinf( angle );
    *cos = std::cosf( angle );
  }


  void fast_sin( float angle, float *const sin )
  {
    RT_DBG_ASSERT( sin != nullptr );

    /*-------------------------------------------------------------------------
    Wrap the angle from -PI to PI
    -------------------------------------------------------------------------*/
    while( angle < -M_PI_F )
    {
      angle += M_2PI_F;
    }

    while( angle > M_PI_F )
    {
      angle -= M_2PI_F;
    }

    /*-------------------------------------------------------------------------
    Compute Sine
    -------------------------------------------------------------------------*/
    *sin = std::sinf( angle );
  }


  float fast_atan2_with_norm( const float y, const float x )
  {
    const float abs_y =
        fabsf( y ) + 1e-20f;    // kludge to prevent 0/0 condition
    float angle;

    if( x >= 0.0f )
    {
      const float r   = ( x - abs_y ) / ( x + abs_y );
      const float rsq = r * r;
      angle           = ( ( 0.1963f * rsq ) - 0.9817f ) * r + ( M_PI_F / 4.0f );
    }
    else
    {
      const float r   = ( x + abs_y ) / ( abs_y - x );
      const float rsq = r * r;
      angle = ( ( 0.1963f * rsq ) - 0.9817f ) * r + ( 3.0f * M_PI_F / 4.0f );
    }

    clear_if_nan( angle );

    if( y < 0.0f )
    {
      return ( -angle );
    }
    else
    {
      return ( angle );
    }
  }


  void park_transform( const float alpha, const float beta, const float theta,
                       float &q, float &d )
  {
    /*-------------------------------------------------------------------------
    Cache the sine/cosine of the angle estimate
    -------------------------------------------------------------------------*/
    float sin, cos;
    fast_sin_cos( theta, &sin, &cos );

    /*-------------------------------------------------------------------------
    Park transform
    -------------------------------------------------------------------------*/
    /* Q-axis aligned */
    // d = ( alpha * cos ) + ( beta * sin );
    // q = ( -alpha * sin ) + ( beta * cos );

    /* D-axis aligned */
    d = ( alpha * cos ) - ( beta * sin );
    q = ( alpha * sin ) + ( beta * cos );
  }


  void inverse_park_transform( const float q, const float d, const float theta,
                               float &a, float &b )
  {
    /*-------------------------------------------------------------------------
    Cache the sine/cosine of the angle estimate
    -------------------------------------------------------------------------*/
    float sin, cos;
    fast_sin_cos( theta, &sin, &cos );

    /*-------------------------------------------------------------------------
    Inverse Park transform
    -------------------------------------------------------------------------*/
    /* Q-axis aligned */
    // a = ( d * cos ) - ( q * sin );
    // b = ( d * sin ) + ( q * cos );

    /* D-axis aligned */
    a = ( d * cos ) + ( q * sin );
    b = ( -d * sin ) + ( q * cos );
  }


  void clarke_transform( const float a, const float b, const float c,
                         float &alpha, float &beta )
  {
    alpha = TWO_OVER_SQRT3 * ( a - 0.5f * b - 0.5f * c );
    beta  = TWO_OVER_SQRT3 * ( SQRT3 / ( 2.0f * ( b - c ) ) );
  }


  void inverse_clarke_transform( const float a, const float b, float &v1,
                                 float &v2, float &v3 )
  {
    v1 = a;
    v2 = -0.5f * a + b * SQRT3_OVER_2;
    v3 = -v1 - v2;
  }


  float clamp( const float v, const float min, const float max )
  {
    return std::max( min, std::min( v, max ) );
  }
}    // namespace Orbit::Control::Math
