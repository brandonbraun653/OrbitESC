/******************************************************************************
 *  File Name:
 *    orbit_filesystem.cpp
 *
 *  Description:
 *    Filesystem interface for Orbit
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/filesystem>
#include <Aurora/logging>
#include <Aurora/memory>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/data/persistent/orbit_database.hpp>

namespace Orbit::Data::File
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = ::Aurora::FileSystem;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static FS::FatFs::Volume                 s_fatfs_volume;
  static Aurora::Memory::Flash::SD::Driver s_sd_driver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void init()
  {
    /*-------------------------------------------------------------------------
    Initialize the FatFs backend
    -------------------------------------------------------------------------*/
    FS::FatFs::initialize();

    /*-------------------------------------------------------------------------
    Initialize the driver configuration
    -------------------------------------------------------------------------*/
    Chimera::SDIO::HWConfig cfg;
    cfg.clear();
    cfg.channel    = IO::SDIO::Channel;
    cfg.clockSpeed = IO::SDIO::ClockSpeed;
    cfg.blockSize  = IO::SDIO::BlockSize;
    cfg.width      = IO::SDIO::BusWidth;
    cfg.clkPin     = IO::SDIO::clkPinInit;
    cfg.cmdPin     = IO::SDIO::cmdPinInit;
    cfg.dxPin[ 0 ] = IO::SDIO::d0PinInit;
    cfg.dxPin[ 1 ] = IO::SDIO::d1PinInit;
    cfg.dxPin[ 2 ] = IO::SDIO::d2PinInit;
    cfg.dxPin[ 3 ] = IO::SDIO::d3PinInit;

    RT_HARD_ASSERT( s_sd_driver.init( cfg ) == true );

    /*-------------------------------------------------------------------------
    Initialize the filesystem volume
    -------------------------------------------------------------------------*/
    s_fatfs_volume.device = &s_sd_driver;
    s_fatfs_volume.path   = "SD";
    RT_HARD_ASSERT( true == FS::FatFs::attachVolume( &s_fatfs_volume ) );

    /*-----------------------------------------------------------------------
    Initialize the Aurora filesystem drivers
    -----------------------------------------------------------------------*/
    bool fs_mounted = true;
    auto intf       = FS::FatFs::getInterface( &s_fatfs_volume );

    LOG_TRACE( "Mounting filesystem" );
    FS::initialize();
    if ( FS::mount( FileSystemMountPoint.cbegin(), intf ) < 0 )
    {
      LOG_TRACE( "Formatting filesystem and remounting" );
      FS::FatFs::formatVolume( &s_fatfs_volume );
      FS::VolumeId mnt_vol = FS::mount( FileSystemMountPoint.cbegin(), intf );

      if ( mnt_vol < 0 )
      {
        fs_mounted = false;
        LOG_ERROR( "Failed to mount filesystem: %d", mnt_vol );
      }
    }

    LOG_TRACE_IF( fs_mounted, "Filesystem mounted" );
  }
}    // namespace Orbit::Data::File
