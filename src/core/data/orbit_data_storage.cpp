/******************************************************************************
 *  File Name:
 *    orbit_data_storage.cpp
 *
 *  Description:
 *    Persistent data storage routines
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <ArduinoJson.h>
#include <Aurora/filesystem>
#include <Aurora/logging>
#include <algorithm>
#include <array>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_internal.hpp>
#include <src/core/data/orbit_data_storage.hpp>
#include <src/core/data/orbit_data_types.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = ::Aurora::FileSystem;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t JSON_FILE_SIZE_MAX = 4096;

  /*-------------------------------------------------------------------
  EEPROM Addressing Limits and Registry Info
  -------------------------------------------------------------------*/
  static constexpr uint16_t EEPROM_IDENTITY_ADDR    = 0x0000;
  static constexpr uint16_t EEPROM_IDENTITY_SZ      = 512;
  static constexpr uint8_t  EEPROM_IDENTITY_VER     = 0;
  static constexpr uint8_t  EEPROM_IDENTITY_TAG     = 0x23;
  static_assert( sizeof( Identity ) <= EEPROM_IDENTITY_SZ );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ParameterNode
  {
    ParamId          id;      /**< Software enumeration tied to parameter */
    etl::string_view key;     /**< String to use in JSON data storage */
    void            *address; /**< Fixed location in memory where data lives */
    size_t           maxSize; /**< Max possible size of the data */
  };
  using ParameterList = std::array<ParameterNode, ParamId::PARAM_COUNT>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  namespace Internal
  {
    static constexpr ParameterList _compile_time_sort( const ParameterList &list )
    {
      auto result = list;
      std::sort( result.begin(), result.end(), []( const ParameterNode &a, const ParameterNode &b ) -> bool { return a.id > b.id; } );
      return result;
    }

    static constexpr ParameterList _unsorted_parameters = {
      /* clang-format off */
      ParameterNode{.id = PARAM_BOOT_COUNT, .key = "boot_count", .address = &SysInfo.bootCount, .maxSize = sizeof( SysInfo.bootCount ) }

      /***** Add new entries above here *****/
      /* clang-format on */
    };
  }    // namespace Internal

  static const ParameterList s_param_info = Internal::_compile_time_sort( Internal::_unsorted_parameters );

  static StaticJsonDocument<JSON_FILE_SIZE_MAX> s_json_cache;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool storeEEPROMCache( const CacheId id )
  {
    /*-------------------------------------------------------------------------
    Select the constraints for updating EEPROM
    -------------------------------------------------------------------------*/
    Aurora::DS::SecureHeader16_t *pObj    = nullptr;
    size_t                        address = 0;
    uint8_t                       version = 0;
    uint8_t                       tag     = 0;
    uint16_t                      size    = 0;

    switch ( id )
    {
      case CacheId::IDENTITY:
        pObj    = reinterpret_cast<Aurora::DS::SecureHeader16_t *>( &SysIdentity );
        address = EEPROM_IDENTITY_ADDR;
        version = EEPROM_IDENTITY_VER;
        tag     = EEPROM_IDENTITY_TAG;
        size    = sizeof( Identity );
        break;

      default:
        return false;
    };

    /*-------------------------------------------------------------------------
    Update the CRC
    -------------------------------------------------------------------------*/
    Aurora::DS::SH::initHeader( pObj, size, version, tag );
    Aurora::DS::SH::addCRC( pObj, size );

    /*-------------------------------------------------------------------------
    Save the data
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( Internal::EepromCtrl );

    auto result = Internal::EepromCtrl.write( address, pObj, size );
    return result == Aurora::Memory::Status::ERR_OK;
  }


  bool loadEEPROMCache( const CacheId id )
  {
    /*-------------------------------------------------------------------------
    Select the constraints for updating EEPROM
    -------------------------------------------------------------------------*/
    Aurora::DS::SecureHeader16_t *pObj    = nullptr;
    size_t                        address = 0;
    uint16_t                      size    = 0;

    switch ( id )
    {
      case CacheId::IDENTITY:
        pObj    = reinterpret_cast<Aurora::DS::SecureHeader16_t *>( &SysIdentity );
        address = EEPROM_IDENTITY_ADDR;
        size    = sizeof( Identity );
        break;

      default:
        return false;
    };

    /*-------------------------------------------------------------------------
    Read the header off first to determine how much data is stored
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( Internal::EepromCtrl );

    auto tmpHdr = Aurora::DS::SecureHeader16_t();
    Internal::EepromCtrl.read( address, &tmpHdr, sizeof( Aurora::DS::SecureHeader16_t ) );

    /* Read the minimal amount of information */
    size = std::min( tmpHdr.size, size );

    /*-------------------------------------------------------------------------
    Load the data
    -------------------------------------------------------------------------*/
    Internal::EepromCtrl.read( address, pObj, size );

    return Aurora::DS::SH::isValid( pObj, size );
  }


  bool loadDisk()
  {
    FS::FileId      fd    = -1;
    FS::AccessFlags flags = ( FS::AccessFlags::O_RDWR | FS::AccessFlags::O_APPEND );
    bool            result = true;

    /*-------------------------------------------------------------------------
    Open the file
    -------------------------------------------------------------------------*/
    if( 0 != FS::fopen( Internal::SystemConfigFile.cbegin(), flags, fd ) )
    {
      LOG_ERROR( "Failed to open configuration file: %s", Internal::SystemConfigFile.cbegin() );
      return false;
    }

    /*-------------------------------------------------------------------------
    Check the size of the file and load default parameters if required
    -------------------------------------------------------------------------*/



    //deserializeJson()
    FS::fclose( fd );
    return result;
  }


  bool flushDisk()
  {
    // serializeJson
    return false;
  }


  bool updateDiskCache( const ParamId param )
  {
    RT_DBG_ASSERT( param < ParamId::PARAM_COUNT );
    return false;
  }


  bool updateDiskCache()
  {
    return false;
  }

}  // namespace Orbit::Data
