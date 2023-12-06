/******************************************************************************
 *  File Name:
 *    serial_config.hpp
 *
 *  Description:
 *    Dynamic serial bus configuration info
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SERIAL_CONFIG_HPP
#define ORBIT_ESC_SERIAL_CONFIG_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/serial>


namespace Orbit::Serial::Config
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Gets the serial port mapped to the command interface
   * @return Chimera::Serial::Driver_rPtr
   */
  Chimera::Serial::Driver_rPtr getCommandPort();

  /**
   * @brief Gets the serial port mapped to the debug console
   * @return Chimera::Serial::Driver_rPtr
   */
  Chimera::Serial::Driver_rPtr getDebugPort();
}  // namespace Orbit::Serial::Config

#endif  /* !ORBIT_ESC_SERIAL_CONFIG_HPP */
