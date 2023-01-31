/******************************************************************************
 *  File Name:
 *    orbit_data_types.hpp
 *
 *  Description:
 *    Type declarations for system data
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SYSTEM_DATA_TYPES_HPP
#define ORBIT_ESC_SYSTEM_DATA_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Aurora/datastruct>
#include <Chimera/common>
#include <src/core/com/serial/serial_interface.pb.h>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class CacheId : uint8_t
  {
    IDENTITY, /**< System identification data (Ver #, Serial #, etc) */
    NUM_OPTIONS
  };

  enum ParamId : uint8_t
  {
    PARAM_BOOT_COUNT = ParamSubId_PARAM_BOOT_COUNT,
    PARAM_COUNT
  };

  /*---------------------------------------------------------------------------
  EEPROM Storage Structures
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
#pragma pack( pop )

  /*---------------------------------------------------------------------------
  System Configuration and Control Structures
  ---------------------------------------------------------------------------*/
  struct Calibration
  {
    void clear()
    {

    }

    void setDefaults()
    {

    }
  };

  struct Controls
  {

    void clear()
    {

    }

    void setDefaults()
    {
    }
  };

  struct Information
  {
    uint32_t bootCount;

    void clear()
    {
      bootCount = 0;
    }

    void setDefaults()
    {
      bootCount = 0;
    }
  };

  struct Configuration
  {
    size_t disk_update_period;

    void clear()
    {
      disk_update_period = 0;
    }

    void setDefaults()
    {
      disk_update_period = DFLT_DISK_SYNC_PERIOD_MS;
    }
  };
}  // namespace Orbit::Data

#endif  /* !ORBIT_ESC_SYSTEM_DATA_TYPES_HPP */
