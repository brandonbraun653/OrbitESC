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
#include <cstddef>
#include <type_traits>

namespace Orbit::Control::Math
{
  /**
   * @brief Generic FIR filter implementation
   *
   * Assumes that the filter coefficients have already been designed and
   * the step() function is being called at the correct frequency.
   *
   * @tparam T      Data type to operate on (float or double)
   * @tparam ORDER  Filter order
   */
  template<typename T, size_t ORDER>
  class FIR
  {
  public:
    using CoefData  = std::array<T, ORDER + 1u>;
    using StateData = std::array<T, ORDER>;

    FIR() : mOutput( 0 ), mCoefficients( {} ), mStates( {} )
    {
    }

    FIR( const CoefData &c ) : mOutput( 0 ), mCoefficients( c ), mStates( {} )
    {
    }

    /**
     * @brief Reset the filter state to idle
     *
     * @param dflt_state  Optional default state of the filter
     */
    void initialize( const T &dflt_state = 0 )
    {
      mOutput = dflt_state;
      mStates.fill( dflt_state );
    }

    /**
     * @brief Steps the filter forward
     * @note Assumes calling at the correct frequency
     *
     * @param u   Next sample to inject
     * @return T  Output of the filter for this step
     */
    T step( const T &u )
    {
      /*-----------------------------------------------------------------------
      Local Variables
      -----------------------------------------------------------------------*/
      T acc1 = 0;
      T zCurr = 0;
      T zNext = u;

      /*-----------------------------------------------------------------------
      Update the filter state
      -----------------------------------------------------------------------*/
      for( size_t n = 0; n < ORDER; n++ )
      {
        zCurr = zNext;
        zNext = mStates[ n ];
        mStates[ n ] = zCurr;

        zCurr *= mCoefficients[ n ];
        acc1 += zCurr;
      }

      zCurr = mCoefficients[ ORDER ] * zNext;
      mOutput = acc1 + zCurr;
      return mOutput;
    }

    /**
     * @brief Get the current value of the filter
     * @return T
     */
    inline T value() const
    {
      return mOutput;
    }

  private:
    T         mOutput;       /**< Current output value */
    CoefData  mCoefficients; /**< Coefficient data */
    StateData mStates;       /**< Current filter state */
  };
}    // namespace Orbit::Control::Math

#endif /* !ORBIT_FILTER_HPP */
