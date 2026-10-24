/******************************************************************************
 *  File Name:
 *    system_control.cpp
 *
 *  Description:
 *    High level system control interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SYSTEM_CONTROL_HPP
#define ORBIT_ESC_SYSTEM_CONTROL_HPP

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the entire system control module
   * @return bool
   */
  void initialize();

}  // namespace Orbit::Control

#endif  /* !ORBIT_ESC_SYSTEM_CONTROL_HPP */
