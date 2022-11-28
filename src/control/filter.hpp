/******************************************************************************
 *  File Name:
 *    filter.hpp
 *
 *  Description:
 *    Discrete filter algorithms
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_FILTER_HPP
#define ORBIT_FILTER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Orbit::Control::Math
{
  template<typename T>
  class LPF
  {
  public:
    T K; /**< Filter gain */

    LPF() : K( 1 ), Tc( 0 ), X( 0 )
    {

    }

    void setTuning( const T tc, const T dt )
    {
      Tc = dt / tc;
    }

    T run( const T u )
    {
      T y = X;
      X = ( ( static_cast<T>( 1 ) - Tc ) * X ) + ( K * Tc * u );

      return y;
    }

  private:
    T Tc; /**< Filter time constant */
    T X;  /**< System state */
  };
}  // namespace Orbit::Control

#endif  /* !ORBIT_FILTER_HPP */
