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
   * @brief Performs a safe shutdown of the system
   */
  void doSafeShutdown();

  /**
   * @brief Logs a fault event with the system
   *
   * @param fault   Fault code
   * @param msg     Message to associate with the fault
   * @return void
   */
  void addFaultEvent( const Fault f, const std::string_view &msg );

  /**
   * @brief Gets the next fault log entry without removing it from the queue
   * @return FaultLogEntry
   */
  FaultLogEntry peekFaultLog();

  /**
   * @brief Pops the next fault log entry off the queue
   * @return void
   */
  void popFaultLog();

}    // namespace Orbit::System

#endif /* !ORBIT_SYSTEM_HPP */
