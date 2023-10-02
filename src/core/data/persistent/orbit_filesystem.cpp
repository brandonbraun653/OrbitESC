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
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t CACHE_SIZE = 256;
  static_assert( CACHE_SIZE % sizeof( uint32_t ) == 0 );

  static constexpr size_t LOOKAHEAD_SIZE = 256;
  static_assert( LOOKAHEAD_SIZE % sizeof( uint32_t ) == 0 );

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

    s_fatfs_volume.device = &s_sd_driver;

    RT_HARD_ASSERT( true == FS::FatFs::attachVolume( &s_fatfs_volume ) );

    /*-----------------------------------------------------------------------
    Initialize the Aurora filesystem drivers
    -----------------------------------------------------------------------*/
    bool fs_mounted = true;    // Assume mounts will succeed. Negate later if failed.
    auto intf       = FS::FatFs::getInterface( &s_fatfs_volume );

    LOG_TRACE( "Mounting filesystem\r\n" );
    FS::initialize();
    if ( FS::mount( FileSystemMountPoint.cbegin(), intf ) < 0 )
    {
      LOG_TRACE( "Formatting filesystem and remounting\r\n" );
      FS::FatFs::formatVolume( &s_fatfs_volume );
      FS::VolumeId mnt_vol = FS::mount( FileSystemMountPoint.cbegin(), intf );

      if ( mnt_vol < 0 )
      {
        fs_mounted = false;
        LOG_ERROR( "Failed to mount filesystem: %d\r\n", mnt_vol );
      }
    }

    LOG_TRACE_IF( fs_mounted, "Filesystem mounted\r\n" );
  }
}    // namespace Orbit::Data::File
