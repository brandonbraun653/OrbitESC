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
#include <src/core/data/persistent/orbit_filesystem.hpp>

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
  static uint32_t        s_read_buffer[ CACHE_SIZE / sizeof( uint32_t ) ];
  static uint32_t        s_prog_buffer[ CACHE_SIZE / sizeof( uint32_t ) ];
  static uint32_t        s_look_buffer[ LOOKAHEAD_SIZE / sizeof( uint32_t ) ];
  static FS::LFS::Volume s_lfs_volume;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void init()
  {
    /*-------------------------------------------------------------------------
    Initialize the LittleFS backend
    -------------------------------------------------------------------------*/
    FS::LFS::initialize();

    auto props = Aurora::Memory::Flash::NOR::getProperties( Aurora::Memory::Flash::NOR::Chip::AT25SF081 );
    s_lfs_volume.clear();

    s_lfs_volume.cfg.read_size        = 64;
    s_lfs_volume.cfg.prog_size        = 64;
    s_lfs_volume.cfg.block_size       = props->blockSize;
    s_lfs_volume.cfg.block_count      = props->endAddress / props->blockSize;
    s_lfs_volume.cfg.cache_size       = CACHE_SIZE;
    s_lfs_volume.cfg.read_buffer      = s_read_buffer;
    s_lfs_volume.cfg.prog_buffer      = s_prog_buffer;
    s_lfs_volume.cfg.lookahead_size   = LOOKAHEAD_SIZE;
    s_lfs_volume.cfg.lookahead_buffer = s_look_buffer;
    s_lfs_volume.cfg.block_cycles     = 500;

#if defined( SIMULATOR )
    s_lfs_volume._dataFile = std::filesystem::current_path() / "orbit_esc_flash.bin";
#endif

    Aurora::Memory::DeviceAttr attr;
    attr.eraseSize = s_lfs_volume.cfg.block_size;
    attr.readSize  = s_lfs_volume.cfg.block_size;
    attr.writeSize = s_lfs_volume.cfg.block_size;

    RT_HARD_ASSERT( true == s_lfs_volume.flash.assignChipSelect( IO::SPI::norCSPort, IO::SPI::norCSPin ) );
    RT_HARD_ASSERT( true == s_lfs_volume.flash.configure( Aurora::Memory::Flash::NOR::Chip::AT25SF081, IO::SPI::spiChannel ) );
    RT_HARD_ASSERT( Aurora::Memory::Status::ERR_OK == s_lfs_volume.flash.open( &attr ) );
    RT_HARD_ASSERT( true == FS::LFS::attachVolume( &s_lfs_volume ) );

    /*-----------------------------------------------------------------------
    Initialize the Aurora filesystem drivers
    -----------------------------------------------------------------------*/
    bool fs_mounted = true;    // Assume mounts will succeed. Negate later if failed.
    auto intf       = FS::LFS::getInterface( &s_lfs_volume );

    LOG_TRACE( "Mounting filesystem\r\n" );
    FS::initialize();
    if ( FS::mount( FileSystemMountPoint.cbegin(), intf ) < 0 )
    {
      LOG_TRACE( "Formatting filesystem and remounting\r\n" );
      FS::LFS::formatVolume( &s_lfs_volume );
      FS::VolumeId mnt_vol = FS::mount( FileSystemMountPoint.cbegin(), intf );

      if ( mnt_vol < 0 )
      {
        fs_mounted = false;
        LOG_ERROR( "Failed to mount filesystem: %d\r\n", mnt_vol );
      }
    }

    LOG_TRACE_IF( fs_mounted, "Filesystem mounted\r\n" );

    /*-------------------------------------------------------------------------
    Create root files if not present yet
    -------------------------------------------------------------------------*/
    FS::FileId      fd    = -1;
    FS::AccessFlags flags = ( FS::AccessFlags::O_RDONLY | FS::AccessFlags::O_CREAT );

    if( fs_mounted )
    {
      RT_HARD_ASSERT( 0 == FS::fopen( SystemConfigFile.cbegin(), flags, fd ) );
      RT_HARD_ASSERT( 0 == FS::fclose( fd ) );

      RT_HARD_ASSERT( 0 == FS::fopen( SystemLogFile.cbegin(), flags, fd ) );
      RT_HARD_ASSERT( 0 == FS::fclose( fd ) );
    }
  }
}  // namespace Orbit::Data::File
