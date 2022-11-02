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
  };

  struct Calibration
  {
    Aurora::DS::SecureHeader16_t header;
  };

  struct Controls
  {
    Aurora::DS::SecureHeader16_t header;
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

  void flushConfigCache( const CacheId id );

  void loadConfigCache(const CacheId id );

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_HPP */
