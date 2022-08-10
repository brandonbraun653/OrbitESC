/******************************************************************************
 *  File Name:
 *    orbit_data.cpp
 *
 *  Description:
 *    Insert Description
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/filesystem>
#include <src/core/data/orbit_data.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Aurora::FileSystem::EEPROM::MBRCache<12, 0x00> s_mbr_cache;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool initialize()
  {
    /*-------------------------------------------------------------------------
    Configure the file system backend
    -------------------------------------------------------------------------*/
    Aurora::FileSystem::EEPROM::FSConfig cfg;
    cfg.address  = 0x53;
    cfg.channel  = IO::I2C::channel;
    cfg.device   = Aurora::Flash::EEPROM::Chip::AT24C02;
    cfg.mbrCache = &s_mbr_cache;

    Aurora::FileSystem::EEPROM::configure( cfg );

    /*-------------------------------------------------------------------------
    Power on the file system
    -------------------------------------------------------------------------*/
    Aurora::FileSystem::attachImplementation( &Aurora::FileSystem::EEPROM::implementation );

    int result = Aurora::FileSystem::mount();
    if( result != 0 )
    {
      LOG_ERROR( "Failed to mount the EEFS file system: %d\r\n", result );
      return false;
    }

    return true;
  }

}    // namespace Orbit::Data
