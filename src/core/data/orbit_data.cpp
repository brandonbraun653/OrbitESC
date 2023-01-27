/******************************************************************************
 *  File Name:
 *    orbit_data.cpp
 *
 *  Description:
 *    Insert Description
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/datastruct>
#include <Aurora/filesystem>
#include <Aurora/logging>
#include <Aurora/memory>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_internal.hpp>
#include <src/core/data/orbit_data_storage.hpp>

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

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Identity    SysIdentity;
  Calibration SysCalibration;
  Controls    SysControl;
  Information SysInfo;

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
  bool initialize()
  {
    /*-------------------------------------------------------------------------
    Reset module memory cache
    -------------------------------------------------------------------------*/
    SysIdentity.clear();
    SysCalibration.clear();
    SysControl.clear();
    SysInfo.clear();

    /*-------------------------------------------------------------------------
    Initialize the EEPROM driver
    -------------------------------------------------------------------------*/
    Aurora::Memory::Flash::EEPROM::DeviceConfig cfg;
    cfg.clear();
    cfg.whichChip     = Aurora::Memory::Flash::EEPROM::Chip::E24LC128;
    cfg.deviceAddress = 0x50;
    cfg.i2cChannel    = IO::I2C::channel;

    RT_HARD_ASSERT( Internal::EepromCtrl.configure( cfg ) );
    RT_HARD_ASSERT( Internal::EepromCtrl.open( nullptr ) == Aurora::Memory::Status::ERR_OK );

    /*-------------------------------------------------------------------------
    Pre-load the cache or initialize to defaults
    -------------------------------------------------------------------------*/
    bool result = true;
    for ( size_t idx = 0; idx < EnumValue( CacheId::NUM_OPTIONS ); idx++ )
    {
      /*-----------------------------------------------------------------------
      Attempt to load previously configured data
      -----------------------------------------------------------------------*/
      CacheId id = static_cast<CacheId>( idx );
      if ( loadEEPROMCache( id ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Prepare the new data to be written
      -----------------------------------------------------------------------*/
      switch ( id )
      {
        case CacheId::IDENTITY:
          SysIdentity.clear();
          break;

        case CacheId::NUM_OPTIONS:
        default:
          RT_DBG_ASSERT( false );
          continue;
      }

      /*-----------------------------------------------------------------------
      Save it off then attempt to read it back to verify it actually made it
      -----------------------------------------------------------------------*/
      bool tmp = storeEEPROMCache( id );
      tmp &= loadEEPROMCache( id );
      LOG_ERROR_IF( tmp == false, "Failed to load cache data %d\r\n", idx );

      result &= tmp;
    }

    return result;
  }


  void bootFileSystem()
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
    if ( FS::mount( Internal::FileSystemMountPoint.cbegin(), intf ) < 0 )
    {
      LOG_TRACE( "Formatting filesystem and remounting\r\n" );
      FS::LFS::formatVolume( &s_lfs_volume );
      FS::VolumeId mnt_vol = FS::mount( "/", intf );

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

    RT_HARD_ASSERT( 0 == FS::fopen( Internal::SystemConfigFile.cbegin(), flags, fd ) );
    RT_HARD_ASSERT( 0 == FS::fclose( fd ) );

    RT_HARD_ASSERT( 0 == FS::fopen( Internal::SystemLogFile.cbegin(), flags, fd ) );
    RT_HARD_ASSERT( 0 == FS::fclose( fd ) );
  }


  void printSystemInfo()
  {
    LOG_INFO( "OrbitESC -- HW: %d, SW:%d.%d.%d, SN:%s\r\n", SysIdentity.hwVersion, SysIdentity.swVerMajor,
              SysIdentity.swVerMinor, SysIdentity.swVerPatch, SysIdentity.serialNumber );
  }

}    // namespace Orbit::Data


namespace Orbit::Data::Internal
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Aurora::Memory::Flash::EEPROM::Driver EepromCtrl;
}
