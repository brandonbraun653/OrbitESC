/******************************************************************************
 *  File Name:
 *    orbit_log_sink.cpp
 *
 *  Description:
 *    Sink driver for the project's on-board NOR filesystem
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>

namespace Orbit::Log
{
  // Need a small cache of data for log messages. Use a ring buffer and then
  // stall the log write until the buffer can be flushed.


}    // namespace Orbit::Log
