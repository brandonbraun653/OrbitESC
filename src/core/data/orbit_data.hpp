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
#include <src/core/data/orbit_data_defaults.hpp>

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

    char boardName[ 16 ];
    char description[ 32 ];
    char serialNumber[ 8 ];
    uint8_t hwVersion;
    uint8_t swVerMajor;
    uint8_t swVerMinor;
    uint8_t swVerPatch;

    void clear()
    {
      header.clear();
      memset( boardName, 0, sizeof( boardName ) );
      memset( description, 0, sizeof( description ) );
      memset( serialNumber, 0, sizeof( serialNumber ) );
      hwVersion  = 0;
      swVerMajor = 0;
      swVerMinor = 0;
      swVerPatch = 0;
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

  /**
   * @brief Prints the system configuration to console
   */
  void printConfiguration();

  /**
   * @brief Periodic function to step through NOR device test sequences
   * @warning Will destroy all existing filesystem data
   */
  void testNORDevice();

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_HPP */
