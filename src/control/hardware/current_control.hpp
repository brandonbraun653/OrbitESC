/******************************************************************************
 *  File Name:
 *    current_control.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CURRENT_CONTROL_HPP
#define ORBIT_CURRENT_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::Control::Field
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  typedef void ( *ISRInnerLoopCallback )( void );

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Operational mode of the OrbitESC FOC Controller
   */
  enum class Mode : uint8_t
  {
    UNKNOWN,    /**< Mode is not known */
    DISABLED,   /**< Control loop is not active */
    OPEN_LOOP,  /**< Control references set, but not actively regulated */
    CLOSED_LOOP /**< Full control with feedback */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the FOC controller hardware and algorithms
   * @return void
   */
  void powerUp();

  /**
   * @brief Powers off the FOC controller hardware and algorithms
   * @return void
   */
  void powerDn();

  /**
   * @brief Set a control mode for the FOC controller
   * @return True if the mode entered successfully
   */
  bool setControlMode( const Mode mode );

  /**
   * @brief Gets the current FOC control mode
   * @return Mode
   */
  Mode getControlMode();

  /**
   * @brief Sets the inner loop callback function.
   *
   * This function will be invoked after all the SVM current control operations have
   * been performed for the current control loop iteration.
   *
   * @param callback  Function to be invoked
   * @return void
   */
  void setInnerLoopCallback( ISRInnerLoopCallback callback );

}    // namespace Orbit::Control::Field

#endif /* !ORBIT_CURRENT_CONTROL_HPP */
