/******************************************************************************
 *  File Name:
 *    orbit_data_storage.hpp
 *
 *  Description:
 *    Persistent data storage layer for OrbitESC
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_STORAGE_HPP
#define ORBIT_DATA_STORAGE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/string.h>
#include <src/core/data/orbit_data_types.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Loads all parameters from disk
   * @return bool
   */
  bool loadDisk();

  /**
   * @brief Immediately serialize all parameters to disk
   * @return bool
   */
  bool flushDisk();

  /**
   * @brief Synchronize any cache updates to disk
   *
   * Should only perform work if the RAM backing store has been changed. This
   * only occurs upon a manual call to updateDiskCache().
   */
  void syncDisk();

  /**
   * @brief Updates cache with a single parameter information
   *
   * @param param   Which parameter to store
   * @return bool
   */
  bool updateDiskCache( const ParamId param );

  /**
   * @brief Update the disk cache for all parameters
   * @return bool
   */
  bool updateDiskCache();

  /**
   * @brief Copies a serialized parameter from the cache to the destination
   *
   * @param param   Which parameter to copy
   * @param dest    Where to copy the data to
   * @param size    How many bytes to copy
   * @return bool   True if success, false if not
   */
  bool copyFromCache( const ParamId param, void *const dest, const size_t size );

  /**
   * @brief Copies a serialized parameter into the cache
   *
   * @param param   Which parameter to update
   * @param src     Where to copy the data from
   * @param size    How many bytes to copy
   * @return bool   True if success, false if not
   */
  bool copyToCache( const ParamId param, const void *const src, const size_t size );

  /**
   * @brief Get storage type of the parameter
   *
   * @param param   Which parameter to query
   * @return ParamType
   */
  ParamType getParamType( const ParamId param );

  /**
   * @brief Checks if the given parameter ID exists
   *
   * @param param   Parameter ID to check
   * @return bool   True if the parameter exists, false if not
   */
  bool paramExists( const int param );

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_STORAGE_HPP */
