/******************************************************************************
 *  File Name:
 *    system.hpp
 *
 *  Description:
 *    High level system behavioral interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SYSTEM_HPP
#define ORBIT_SYSTEM_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/system_types.hpp>

namespace Orbit::System
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Gets the currently executing mode of the system
   * @return Mode
   */
  Mode getMode();

  /**
   * @brief Safely shuts down the system and transitions to a new mode
   * @return void
   */
  void setMode( const Mode next );

  /**
   * @brief Converts a mode enum to a string
   *
   * @param mode  Mode to convert
   * @return const char*
   */
  const char *modeString( const Mode mode );

  /**
   * @brief Performs a safe shutdown of the system
   */
  void doSafeShutdown();

}    // namespace Orbit::System

#endif /* !ORBIT_SYSTEM_HPP */
