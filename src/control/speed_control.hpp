/******************************************************************************
 *  File Name:
 *    speed_control.hpp
 *
 *  Description:
 *    Interface for the motor speed controller
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SPEED_CONTROL_HPP
#define ORBIT_SPEED_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::Control::Speed
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Operational mode of the OrbitESC Speed Controller
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
   * @brief Powers up the speed controller hardware and algorithms
   * @return void
   */
  void powerUp();

  /**
   * @brief Set a control mode for the speed controller
   * @return True if the mode entered successfully
   */
  bool setControlMode( const Mode mode );

  /**
   * @brief Gets the current control mode
   * @return Mode
   */
  Mode getControlMode();

}    // namespace Orbit::Control::Speed

#endif /* !ORBIT_SPEED_CONTROL_HPP */
