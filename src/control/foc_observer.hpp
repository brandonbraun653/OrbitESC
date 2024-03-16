/******************************************************************************
 *  File Name:
 *    foc_observer.hpp
 *
 *  Description:
 *    Observer implementation for the Field Oriented Control (FOC) system.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FOC_OBSERVER_HPP
#define ORBIT_ESC_FOC_OBSERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/foc_data.hpp>

namespace Orbit::Control::Observer
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class Policy : uint8_t
  {
    NONE,
    LUENBERGER,
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct Input
  {
    float dt;     /**< Observer sampling delta step in seconds */
    float iAlpha; /**< Measured stator current in the alpha frame */
    float iBeta;  /**< Measured stator current in the beta frame */
    float vAlpha; /**< Measured stator voltage in the alpha frame */
    float vBeta;  /**< Measured stator voltage in the beta frame */
  };

  struct Output
  {
    float theta;  /**< Estimated electrical position in radians */
    float omega;  /**< Estimated electrical rotational speed in rad/s */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the observer module
   * @note This should be called once at system startup
   */
  void initialize();

  /**
   * @brief Assigns a new policy to execute for the observer model
   *
   * @param policy   The new observer policy to use
   */
  void setPolicy( const Policy policy );

  /**
   * @brief Takes a single step of the observer policy
   *
   * @param input   The input data to the observer
   * @param output  The output data from the observer
   */
  void execute( const Input &input, Output &output );

  /**
   * @brief Resets the current observer to its initial state
   */
  void reset();

  /**
   * @brief Retrieves the latest estimates from the observer.
   * @note This is a non-atomic function, so the data may be in an inconsistent state.
   *
   * @return Output
   */
  Output estimates();

}  // namespace Orbit::Control::Observer

#endif  /* !ORBIT_ESC_FOC_OBSERVER_HPP */
