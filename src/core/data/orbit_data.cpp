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
#include <Aurora/memory>
#include <Aurora/filesystem>
#include <src/core/data/orbit_data.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = ::Aurora::FileSystem;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  #define FS_MOUNT_PATH   ""

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Aurora::Flash::EEPROM::Driver sEEPROMFlash; /**< Flash memory driver supporting the file system */
  static FS::LFS::Volume               s_lfs_volume; /**< LittleFS volume for the NOR flash chip */
  static bool                          s_fs_mounted; /**< Checks if the filesystem mounted OK */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize the EEPROM driver
    -------------------------------------------------------------------------*/
    // Aurora::Flash::EEPROM::DeviceConfig cfg;
    // cfg.clear();
    // cfg.whichChip     = Aurora::Flash::EEPROM::Chip::AT24C02;
    // cfg.deviceAddress = 0x53;
    // cfg.i2cChannel    = IO::I2C::channel;

    // RT_HARD_ASSERT( sEEPROMFlash.configure( cfg ) );

    /*-------------------------------------------------------------------------
    Power on the file system
    -------------------------------------------------------------------------*/
    return true;
  }


  void bootFileSystem()
  {
    /*-------------------------------------------------------------------------
    Reset module memory
    -------------------------------------------------------------------------*/
    s_fs_mounted = true;  // Assume mounts will succeed. Negate later if failed.

    /*-------------------------------------------------------------------------
    Initialize the LittleFS backend
    -------------------------------------------------------------------------*/
    FS::LFS::initialize();

    auto props = Aurora::Flash::NOR::getProperties( Aurora::Flash::NOR::Chip::AT25SF081 );
    s_lfs_volume.clear();

    s_lfs_volume.cfg.read_size      = 16;
    s_lfs_volume.cfg.prog_size      = 16;
    s_lfs_volume.cfg.block_size     = props->blockSize;
    s_lfs_volume.cfg.block_count    = props->endAddress / props->blockSize;
    s_lfs_volume.cfg.cache_size     = 64;
    s_lfs_volume.cfg.lookahead_size = 16;
    s_lfs_volume.cfg.block_cycles   = 500;

    RT_HARD_ASSERT( true == s_lfs_volume.flash.configure( Aurora::Flash::NOR::Chip::AT25SF081, IO::SPI::spiChannel ) );
    RT_HARD_ASSERT( true == FS::LFS::attachVolume( &s_lfs_volume ) );

    /*-----------------------------------------------------------------------
    Initialize the Aurora filesystem drivers
    -----------------------------------------------------------------------*/
    auto intf    = FS::LFS::getInterface();
    intf.context = &s_lfs_volume;

    FS::initialize();

    if( FS::mount( FS_MOUNT_PATH, intf ) < 0 )
    {
      FS::LFS::formatVolume( &s_lfs_volume );
      FS::VolumeId mnt_vol = FS::mount( FS_MOUNT_PATH, intf );

      if( mnt_vol < 0 )
      {
        s_fs_mounted = false;
      }
    }

    LOG_ERROR_IF( s_fs_mounted == false, "Failed to mount filesystem\r\n" );
  }

}    // namespace Orbit::Data
