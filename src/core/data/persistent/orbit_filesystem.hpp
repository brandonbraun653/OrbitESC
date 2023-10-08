/******************************************************************************
 *  File Name:
 *    orbit_filesystem.hpp
 *
 *  Description:
 *    Filesystem interface for Orbit
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_FILESYSTEM_HPP
#define ORBIT_FILESYSTEM_HPP

namespace Orbit::Data::FileSystem
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Prepares the filesystem for use
   * @return void
   */
  void init();

  /**
   * @brief Checks if the filesystem is mounted and ready for use
   *
   * @return bool
   */
  bool isMounted();

}  // namespace Orbit::Data::File

#endif  /* !ORBIT_FILESYSTEM_HPP */
