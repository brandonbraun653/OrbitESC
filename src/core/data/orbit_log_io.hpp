/******************************************************************************
 *  File Name:
 *    orbit_log_io.hpp
 *
 *  Description:
 *    IO operations associated with inspecting the log files
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
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
  static constexpr std::string_view LogFile = "system_events.log";

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  int initialize();

  int enable();

  int disable();

  void flushCache();

  void dumpToConsole();

}  // namespace Orbit::Log

#endif  /* !ORBIT_LOG_IO_HPP */
