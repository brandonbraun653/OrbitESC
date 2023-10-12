/******************************************************************************
 *  File Name:
 *    orbit_log_io.hpp
 *
 *  Description:
 *    IO operations associated with inspecting the log files
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_LOG_IO_HPP
#define ORBIT_LOG_IO_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <string_view>

namespace Orbit::Log
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr std::string_view LogFile = "/system_events.log";
  static constexpr std::string_view TestFile = "/test.log";

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the logger
   * @return int
   */
  int initialize();

  /**
   * @brief Enables the event logger
   * @return int
   */
  int enable();

  /**
   * @brief Disables the event logger
   * @return int
   */
  int disable();

  /**
   * @brief Flushes any cached messages to the logger
   */
  void flushCache();

  /**
   * @brief Dumps the entire contents of the log to console
   */
  void dumpToConsole();

  /**
   * @brief Logs a message to the test log file
   *
   * @param msg Message to log
   * @return bool True if the message was logged, false otherwise
   */
  bool logTestMessage( const std::string_view msg );

}  // namespace Orbit::Log

#endif  /* !ORBIT_LOG_IO_HPP */
