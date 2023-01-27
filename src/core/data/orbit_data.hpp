/******************************************************************************
 *  File Name:
 *    orbit_data.hpp
 *
 *  Description:
 *    Orbit data storage interface
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_HPP
#define ORBIT_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Aurora/datastruct>
#include <Chimera/common>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/data/orbit_data_types.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern Identity    SysIdentity;
  extern Calibration SysCalibration;
  extern Controls    SysControl;
  extern Information SysInfo;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the data storage system
   * @return bool
   */
  bool initialize();

  /**
   * @brief Powers on the file system and prepares it for use
   */
  void bootFileSystem();

  /**
   * @brief Prints the system configuration to console
   */
  void printSystemInfo();

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_HPP */
