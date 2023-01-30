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
   * @brief Flushes one of the cached data structures to disk
   *
   * @param id      Which cache item to save
   * @return bool   True if success, false if not
   */
  bool storeEEPROMCache( const CacheId id );

  /**
   * @brief Loads one of the cached data structures from disk
   *
   * @param id      Which cache item to load
   * @return bool   True if success, false if not
   */
  bool loadEEPROMCache( const CacheId id );

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

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_STORAGE_HPP */
