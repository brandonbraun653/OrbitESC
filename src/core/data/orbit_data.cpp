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
#include <Aurora/datastruct>
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
  static constexpr size_t CACHE_SIZE = 256;
  static_assert( CACHE_SIZE % sizeof( uint32_t ) == 0 );

  static constexpr size_t LOOKAHEAD_SIZE = 256;
  static_assert( LOOKAHEAD_SIZE % sizeof( uint32_t ) == 0 );

  /*-------------------------------------------------------------------
  EEPROM Addressing Limits
  -------------------------------------------------------------------*/
  static constexpr uint16_t EEPROM_IDENTITY_ADDR = 0x0000;
  static constexpr uint16_t EEPROM_IDENTITY_SZ   = 512;

  static constexpr uint16_t EEPROM_CALIBRATION_ADDR = EEPROM_IDENTITY_ADDR + EEPROM_IDENTITY_SZ;
  static constexpr uint16_t EEPROM_CALIBRATION_SZ   = 4096;

  static constexpr uint16_t EEPROM_CONTROLS_ADDR = EEPROM_CALIBRATION_ADDR + EEPROM_CALIBRATION_SZ;
  static constexpr uint16_t EEPROM_CONTROLS_SZ   = 4096;

  /*---------------------------------------------------------------------------
  Assertions
  ---------------------------------------------------------------------------*/
  static_assert( sizeof( Identity ) <= EEPROM_IDENTITY_SZ );

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Identity    SysIdentity;
  Calibration SysCalibration;
  Controls    SysControl;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static uint32_t                      s_read_buffer[ CACHE_SIZE / sizeof( uint32_t ) ];
  static uint32_t                      s_prog_buffer[ CACHE_SIZE / sizeof( uint32_t ) ];
  static uint32_t                      s_look_buffer[ LOOKAHEAD_SIZE / sizeof( uint32_t ) ];
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

    // RT_HARD_LOG( sEEPROMFlash.configure( cfg ) );

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
    s_fs_mounted = true;    // Assume mounts will succeed. Negate later if failed.

    /*-------------------------------------------------------------------------
    Initialize the LittleFS backend
    -------------------------------------------------------------------------*/
    FS::LFS::initialize();

    auto props = Aurora::Flash::NOR::getProperties( Aurora::Flash::NOR::Chip::AT25SF081 );
    s_lfs_volume.clear();

    s_lfs_volume.cfg.read_size        = CACHE_SIZE;
    s_lfs_volume.cfg.prog_size        = CACHE_SIZE;
    s_lfs_volume.cfg.block_size       = props->blockSize;
    s_lfs_volume.cfg.block_count      = props->endAddress / props->blockSize;
    s_lfs_volume.cfg.cache_size       = CACHE_SIZE;
    s_lfs_volume.cfg.read_buffer      = s_read_buffer;
    s_lfs_volume.cfg.prog_buffer      = s_prog_buffer;
    s_lfs_volume.cfg.lookahead_size   = LOOKAHEAD_SIZE;
    s_lfs_volume.cfg.lookahead_buffer = s_look_buffer;
    s_lfs_volume.cfg.block_cycles     = 500;

    RT_HARD_ASSERT( true == s_lfs_volume.flash.assignChipSelect( IO::SPI::norCSPort, IO::SPI::norCSPin ) );
    RT_HARD_ASSERT( true == s_lfs_volume.flash.configure( Aurora::Flash::NOR::Chip::AT25SF081, IO::SPI::spiChannel ) );
    RT_HARD_ASSERT( true == FS::LFS::attachVolume( &s_lfs_volume ) );

    s_lfs_volume.flash.erase();

    /*-----------------------------------------------------------------------
    Initialize the Aurora filesystem drivers
    -----------------------------------------------------------------------*/
    auto intf = FS::LFS::getInterface( &s_lfs_volume );

    FS::initialize();
    if ( FS::mount( "", intf ) < 0 )
    {
      LOG_TRACE( "Formatting filesystem and remounting\r\n" );
      FS::LFS::formatVolume( &s_lfs_volume );
      FS::VolumeId mnt_vol = FS::mount( "", intf );

      if ( mnt_vol < 0 )
      {
        s_fs_mounted = false;
        LOG_ERROR( "Failed to mount filesystem: %d\r\n", mnt_vol );
      }
    }

    LOG_TRACE_IF( s_fs_mounted, "Filesystem mounted\r\n" );
  }

}    // namespace Orbit::Data
