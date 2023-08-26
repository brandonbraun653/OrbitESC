/******************************************************************************
 *  File Name:
 *    foc_motor.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_FOC_MOTOR_HPP
#define ORBIT_FOC_MOTOR_HPP

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
   * @brief Set a control mode for the FOC controller
   * @return True if the mode entered successfully
   */
  bool setControlMode( const Mode mode );

  /**
   * @brief Gets the current FOC control mode
   * @return Mode
   */
  Mode getControlMode();

}    // namespace Orbit::Control::Field

#endif /* !ORBIT_FOC_MOTOR_HPP */
