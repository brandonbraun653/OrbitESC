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
#include <Chimera/function>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/data/persistent/orbit_database.hpp>
#include <src/core/hw/orbit_sdio.hpp>
#include <src/core/system.hpp>

namespace Orbit::Data::FileSystem
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = ::Aurora::FileSystem;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static FS::FatFs::Volume                 s_fatfs_volume; /**< FatFs specific info */
  static FS::VolumeId                      s_mounted_vol;  /**< Volume ID of the mounted filesystem */
  static Aurora::Memory::Flash::SD::Driver s_sd_driver;    /**< Flash translation layer to SD card */


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Callback for when the SD card is inserted
   *
   * @param unused  Not used
   * @return void
   */
  static void onCardInsert( void *unused )
  {
    bool fs_mounted = true;
    auto intf       = FS::FatFs::getInterface( &s_fatfs_volume );

    /*-------------------------------------------------------------------------
    Try mounting a few times. I've noticed that sometimes the card doesn't get
    detected properly on the first try for some reason.
    -------------------------------------------------------------------------*/
    LOG_DEBUG( "SD card inserted. Mounting..." );
    size_t attempts = 0;
    while ( attempts < 3 && s_mounted_vol < 0 )
    {
      s_mounted_vol = FS::mount( FileSystemMountPoint.cbegin(), intf );
      Chimera::delayMilliseconds( 100 );
      attempts++;
    }

    /*-------------------------------------------------------------------------
    If the mount failed, try formatting the card and mounting again
    -------------------------------------------------------------------------*/
    if ( s_mounted_vol < 0 )
    {
      LOG_DEBUG( "Failed initial %d attempts. Formatting SD card and remounting", attempts );
      FS::FatFs::formatVolume( &s_fatfs_volume );
      s_mounted_vol = FS::mount( FileSystemMountPoint.cbegin(), intf );

      if ( s_mounted_vol < 0 )
      {
        fs_mounted = false;
        LOG_ERROR( "Failed to mount SD card. Error: %d", s_mounted_vol );
        System::addFaultEvent( System::Fault::FS_MOUNT_FAILED, "Failed to mount SD card" );
      }
    }

    LOG_DEBUG_IF( fs_mounted, "SD card mounted" );
  }


  /**
   * @brief Callback for when the SD card is removed
   *
   * @param unused  Not used
   * @return void
   */
  static void onCardRemove( void *unused )
  {
    /*-------------------------------------------------------------------------
    If the filesystem isn't mounted, then there's nothing to do
    -------------------------------------------------------------------------*/
    if ( s_mounted_vol < 0 )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Unmount the filesystem
    -------------------------------------------------------------------------*/
    LOG_DEBUG( "SD card removed" );
    FS::unmount( s_mounted_vol );
    s_mounted_vol = -1;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void init()
  {
    /*-------------------------------------------------------------------------
    Initialize the filesystem FatFs backend
    -------------------------------------------------------------------------*/
    FS::FatFs::initialize();

    /*-------------------------------------------------------------------------
    Initialize the filesystem layer
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_sd_driver.init( IO::SDIO::Channel ) == true );

    s_fatfs_volume.device = &s_sd_driver;
    s_fatfs_volume.path   = "SD";
    RT_HARD_ASSERT( true == FS::FatFs::attachVolume( &s_fatfs_volume ) );

    FS::initialize();

    /*-------------------------------------------------------------------------
    Initialize module data
    -------------------------------------------------------------------------*/
    s_mounted_vol = -1;

    /*-------------------------------------------------------------------------
    Bind the mount/unmount behavior to the SDIO card detect pin
    -------------------------------------------------------------------------*/
    auto insert_cb = Chimera::Function::vGeneric::create<onCardInsert>();
    SDIO::onCardInsert( insert_cb );

    auto remove_cb = Chimera::Function::vGeneric::create<onCardRemove>();
    SDIO::onCardRemove( remove_cb );

    /*-------------------------------------------------------------------------
    Mount the filesystem if we have a card inserted on power up
    -------------------------------------------------------------------------*/
    if ( SDIO::isCardPresent() )
    {
      onCardInsert( nullptr );
    }
  }


  bool isMounted()
  {
    return s_mounted_vol >= 0;
  }

}    // namespace Orbit::Data::File
