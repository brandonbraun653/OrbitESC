/******************************************************************************
 *  File Name:
 *    current_control.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
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
   * @brief Sets the input references for the inner loop current controller.
   * @note Only active in manual (open loop) control mode.
   *
   * @param iq_ref  Current reference in the q-axis
   * @param id_ref  Current reference in the d-axis
   * @param theta   Rotor angle in radians
   * @return void
   */
  void setInnerLoopReferences( const float iq_ref, const float id_ref, const float theta );

}    // namespace Orbit::Control::Field

#endif /* !ORBIT_CURRENT_CONTROL_HPP */
