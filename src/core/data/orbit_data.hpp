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
#include <Aurora/datastruct>
#include <Chimera/common>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class CacheId : uint8_t
  {
    IDENTITY,
    CALIBRATION,
    CONTROL_SYSTEM,

    NUM_OPTIONS
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
#pragma pack( push, 1 )
  struct Identity
  {
    Aurora::DS::SecureHeader16_t header;

    void clear()
    {

    }
  };

  struct Calibration
  {
    Aurora::DS::SecureHeader16_t header;


    void clear()
    {

    }
  };

  struct Controls
  {
    Aurora::DS::SecureHeader16_t header;

    void clear()
    {

    }
  };
#pragma pack( pop )

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern Identity    SysIdentity;
  extern Calibration SysCalibration;
  extern Controls    SysControl;

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

  /**
   * @brief Flushes one of the cached data structures to disk
   *
   * @param id      Which cache item to save
   * @return bool   True if success, false if not
   */
  bool saveConfigCache( const CacheId id );

  /**
   * @brief Loads one of the cached data structures from disk
   *
   * @param id      Which cache item to load
   * @return bool   True if success, false if not
   */
  bool loadConfigCache(const CacheId id );

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_HPP */
