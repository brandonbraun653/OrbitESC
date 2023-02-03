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
#include <Chimera/thread>
#include <algorithm>
#include <array>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_internal.hpp>
#include <src/core/data/orbit_data_storage.hpp>
#include <src/core/data/orbit_data_types.hpp>
#include <nanoprintf.h>
#include <etl/to_arithmetic.h>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = ::Aurora::FileSystem;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr FS::AccessFlags FILE_FLAGS         = FS::AccessFlags::O_RDWR | FS::AccessFlags::O_CREAT;
  static constexpr size_t          JSON_FILE_SIZE_MAX = 1024;
  static constexpr const char     *FMT_UINT32         = "%d";

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
    ParameterId          id;      /**< Software enumeration tied to parameter */
    const char      *key;     /**< String to use in JSON data storage */
    const char      *fmt;     /**< String format for serialization */
    void            *address; /**< Fixed location in memory where data lives */
    size_t           maxSize; /**< Max possible size of the data */
  };
  using ParameterList = std::array<ParameterNode, ParameterId::PARAM_COUNT>;

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
      ParameterNode{.id = PARAM_BOOT_COUNT, .key = "boot_count", .fmt = FMT_UINT32, .address = &SysInfo.bootCount, .maxSize = sizeof( SysInfo.bootCount ) }

      /***** Add new entries above here *****/
      /* clang-format on */
    };
  }    // namespace Internal

  static const ParameterList s_param_info = Internal::_compile_time_sort( Internal::_unsorted_parameters );

  static StaticJsonDocument<JSON_FILE_SIZE_MAX> s_json_cache;
  static Chimera::Thread::RecursiveMutex        s_json_lock;
  static etl::array<char, 64>                   s_fmt_buffer;
  static bool                                   s_json_pend_changes;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void load_defaults()
  {
    SysCalibration.setDefaults();
    SysControl.setDefaults();
    SysInfo.setDefaults();
    SysConfig.setDefaults();
  }

  static void deserialize_disk_cache()
  {
    for( size_t idx = 0; idx < ParameterId::PARAM_COUNT; idx++ )
    {
      /*-----------------------------------------------------------------------
      Validate the key exists in the document
      -----------------------------------------------------------------------*/
      const ParameterNode *node = &s_param_info[ idx ];
      if( !s_json_cache.containsKey( node->key ) )
      {
        LOG_WARN( "Configuration backing store missing key: %s", node->key );
        continue;
      }

      /*-----------------------------------------------------------------------
      Decode according to the marked format
      -----------------------------------------------------------------------*/
      if( node->fmt == FMT_UINT32 )
      {
        uint32_t *val = static_cast<uint32_t *>( node->address );
        *val = s_json_cache[ node->key ].as<uint32_t>();
      }
      else
      {
        LOG_WARN( "Unsupported format specifier for key: %s", node->key );
      }
    }
  }

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
    Chimera::Thread::LockGuard _lck( s_json_lock );

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    FS::FileId      fd    = -1;
    s_json_pend_changes   = false;

    /*-------------------------------------------------------------------------
    Always load the default configuration as a fallback
    -------------------------------------------------------------------------*/
    load_defaults();

    /*-------------------------------------------------------------------------
    Open the file
    -------------------------------------------------------------------------*/
    LOG_INFO( "Loading configuration from disk..." );
    if( 0 != FS::fopen( Internal::SystemConfigFile.cbegin(), FILE_FLAGS, fd ) )
    {
      LOG_ERROR( "Failed to open configuration file: %s", Internal::SystemConfigFile.cbegin() );
      return false;
    }

    /*-------------------------------------------------------------------------
    Check the size of the file and load default parameters if required
    -------------------------------------------------------------------------*/
    size_t file_size = FS::fsize( fd );

    if( file_size > JSON_FILE_SIZE_MAX )
    {
      LOG_ERROR( "Not enough memory to load file of size %d bytes", file_size );
      FS::fclose( fd );
      return false;
    }
    else if( file_size == 0 )
    {
      LOG_INFO( "Empty configuration. Loading defaults and syncing with disk." );
      FS::fclose( fd );
      RT_HARD_ASSERT( updateDiskCache() );
      RT_HARD_ASSERT( flushDisk() );
    }

    /*-------------------------------------------------------------------------
    Read the raw data onto the stack. Unfortunately the JSON library doesn't
    handle chunking very well (depends on manual JSON object boundary parsing).
    OrbitESC is unlikely to have massive amounts of configuration data, so this
    will work in a pinch. Several tasks already have large stacks anyways.
    -------------------------------------------------------------------------*/
    char file_data[ JSON_FILE_SIZE_MAX ];
    memset( file_data, 0, sizeof( file_data ) );

    FS::frewind( fd );
    size_t read_bytes = FS::fread( file_data, 1u, file_size, fd );
    RT_HARD_ASSERT( read_bytes == file_size );
    FS::fclose( fd );

    /*-------------------------------------------------------------------------
    Parse the JSON information using the copy-on-write interface
    https://arduinojson.org/v6/api/json/deserializejson/
    -------------------------------------------------------------------------*/
    const char *copy_ptr = file_data;
    DeserializationError error = deserializeJson( s_json_cache, copy_ptr );
    if( error )
    {
      LOG_ERROR( "Failed json decode: %s", error.c_str() );
      return false;
    }

    deserialize_disk_cache();
    return true;
  }


  bool flushDisk()
  {
    Chimera::Thread::LockGuard _lck( s_json_lock );

    /*-------------------------------------------------------------------------
    Take whatever is in the cache right now and serialize it to disk
    -------------------------------------------------------------------------*/
    char file_data[ JSON_FILE_SIZE_MAX ];
    memset( file_data, 0, sizeof( file_data ) );
    const size_t serialized_size = serializeJson( s_json_cache, file_data, sizeof( file_data ) );

    /*-------------------------------------------------------------------------
    Flush the data to disk
    -------------------------------------------------------------------------*/
    LOG_INFO( "Storing configuration to disk..." );
    FS::FileId fd       = -1;
    s_json_pend_changes = false;

    if ( 0 == FS::fopen( Internal::SystemConfigFile.cbegin(), FILE_FLAGS, fd ) )
    {
      FS::frewind( fd );
      const size_t written = FS::fwrite( file_data, 1u, serialized_size, fd );
      FS::fclose( fd );
      LOG_ERROR_IF( written != serialized_size, "Failed to flush all bytes to disk. Act:%d, Exp:%d", written, serialized_size );
      return written == serialized_size;
    }
    else
    {
      LOG_ERROR( "Disk flush failed. Cannot open file." );
      return false;
    }
  }


  void syncDisk()
  {
    if( s_json_pend_changes )
    {
      auto result = flushDisk();
      LOG_ERROR_IF( result == false, "Failed disk sync" );
    }
  }


  bool updateDiskCache( const ParameterId param )
  {
    Chimera::Thread::LockGuard _lck( s_json_lock );

    RT_DBG_ASSERT( param < s_param_info.size() );
    const ParameterNode *node = &s_param_info[ param ];

    s_fmt_buffer.fill( 0 );

    /*-------------------------------------------------------------------------
    Serialize the data to string for storage
    -------------------------------------------------------------------------*/
    if( node->fmt == FMT_UINT32 )
    {
      RT_DBG_ASSERT( node->maxSize == sizeof( uint32_t ) );
      auto val = reinterpret_cast<uint32_t*>( node->address );
      npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), node->fmt, *( val ) );
    }
    else
    {
      /* Missing format specifier */
      RT_DBG_ASSERT( false );
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the JSON document cache with the value
    -------------------------------------------------------------------------*/
    s_json_cache[ node->key ] = s_fmt_buffer.data();
    s_json_pend_changes       = true;
    return true;
  }


  bool updateDiskCache()
  {
    Chimera::Thread::LockGuard _lck( s_json_lock );

    bool sticky_result = true;
    for( size_t idx = 0; idx < ParameterId::PARAM_COUNT; idx++ )
    {
      sticky_result &= updateDiskCache( static_cast<ParameterId>( idx ) );
    }

    return sticky_result;
  }

}  // namespace Orbit::Data
