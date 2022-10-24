/******************************************************************************
 *  File Name:
 *    orbit_data.hpp
 *
 *  Description:
 *    Orbit data storage interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_HPP
#define ORBIT_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/common>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the data storage system
   *
   * @return bool
   */
  bool initialize();

  /**
   * @brief Powers on the file system and prepares it for use
   */
  void bootFileSystem();

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_HPP */
