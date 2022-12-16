/******************************************************************************
 *  File Name:
 *    serial_runtime.hpp
 *
 *  Description:
 *    Serial runtime operations for processing debug interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_RUNTIME_HPP
#define ORBIT_SERIAL_RUNTIME_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime();

  void processSerial();
}  // namespace Orbit::Serial

#endif  /* !ORBIT_SERIAL_RUNTIME_HPP */
