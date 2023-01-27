/******************************************************************************
 *  File Name:
 *    orbit_data_internal.hpp
 *
 *  Description:
 *    Internal control interface to Orbit data layer
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_INTERNAL_HPP
#define ORBIT_DATA_INTERNAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/memory>
#include <etl/string.h>

namespace Orbit::Data::Internal
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  /**
   * @brief Core EEPROM driver for persistent but rarely changing data
   */
  extern Aurora::Memory::Flash::EEPROM::Driver EepromCtrl;

  /**
   * @brief Dynamic mount point for the file system driver
   */
  static const etl::string_view FileSystemMountPoint = "/";

  /**
   * @brief Name of the file used internally to back our key-value store
   */
  static const etl::string_view SystemConfigFile = "/sys_config.json";

  /**
   * @brief Name of the file used internally to store logs to
   */
  static const etl::string_view SystemLogFile = "/sys_log.txt";

}    // namespace Orbit::Data::Internal

#endif /* !ORBIT_DATA_INTERNAL_HPP */
