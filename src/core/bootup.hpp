/******************************************************************************
 *  File Name:
 *    bootup.hpp
 *
 *  Description:
 *    System bootup procedures
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_BOOT_UP_HPP
#define ORBIT_ESC_BOOT_UP_HPP

namespace Orbit::Boot
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes system drivers used in the project
   *
   * Powers on the peripherals described in board_map.hpp. Upon completion, the
   * system will be ready to start consuming the peripherals per normal.
   */
  void powerUpSystemDrivers();

  /**
   * @brief Starts a cascade of tasks to jump into a normal execution flow
   *
   * Not all tasks may start immediately, but once this function is called, it's
   * expected that the system will transition from the power up stage into normal
   * execution.
   */
  void startTasks();

}  // namespace Orbit::Boot

#endif  /* !ORBIT_ESC_BOOT_UP_HPP */
