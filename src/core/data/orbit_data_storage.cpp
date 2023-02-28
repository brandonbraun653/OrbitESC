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
#include <Chimera/system>
#include <Chimera/thread>
#include <algorithm>
#include <array>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_internal.hpp>
#include <src/core/data/orbit_data_storage.hpp>
#include <src/core/data/orbit_data_types.hpp>
#include <nanoprintf.h>
#include <etl/algorithm.h>
#include <etl/crc32.h>

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
  static constexpr const char     *FMT_UINT8          = "%u";
  static constexpr const char     *FMT_UINT16         = FMT_UINT8;
  static constexpr const char     *FMT_UINT32         = "%lu";
  static constexpr const char     *FMT_FLOAT          = "%4.9f";
  static constexpr const char     *FMT_DOUBLE         = "%4.17f";
  static constexpr const char     *FMT_STRING         = "%s";
  static constexpr const char     *FMT_BOOL           = "%d";

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ParameterNode
  {
    ParamId     id;      /**< Software enumeration tied to parameter */
    ParamType   type;    /**< Type of data stored in the parameter */
    const char *key;     /**< String to use in JSON data storage */
    void       *address; /**< Fixed location in memory where data lives */
    size_t      maxSize; /**< Max possible size of the data */
  };
  using ParameterList = std::array<ParameterNode, 10>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  namespace Internal
  {
    static constexpr ParameterList _compile_time_sort( const ParameterList &list )
    {
      auto result = list;
      std::sort( result.begin(), result.end(),
                 []( const ParameterNode &a, const ParameterNode &b ) -> bool { return a.id < b.id; } );
      return result;
    }

    static constexpr ParameterList _unsorted_parameters = {
      /* clang-format off */
      /*-----------------------------------------------------------------------
      Read Only Parameters
      -----------------------------------------------------------------------*/
      ParameterNode{ .id = ParamId_PARAM_BOOT_COUNT,    .type = ParamType_UINT32, .key = "pwr_cnt", .address = &SysInfo.bootCount,        .maxSize = sizeof( SysInfo.bootCount )       },
      ParameterNode{ .id = ParamId_PARAM_HW_VERSION,    .type = ParamType_UINT8,  .key = "hw_ver",  .address = &SysIdentity.hwVersion,    .maxSize = sizeof( SysIdentity.hwVersion )   },
      ParameterNode{ .id = ParamId_PARAM_SW_VERSION,    .type = ParamType_STRING, .key = "sw_ver",  .address = &SysIdentity.swVersion,    .maxSize = SysIdentity.swVersion.MAX_SIZE    },
      ParameterNode{ .id = ParamId_PARAM_DEVICE_ID,     .type = ParamType_UINT32, .key = "dev_id",  .address = &SysIdentity.deviceId,     .maxSize = sizeof( SysIdentity.deviceId )    },
      ParameterNode{ .id = ParamId_PARAM_BOARD_NAME,    .type = ParamType_STRING, .key = "name",    .address = &SysIdentity.boardName,    .maxSize = SysIdentity.boardName.MAX_SIZE    },
      ParameterNode{ .id = ParamId_PARAM_DESCRIPTION,   .type = ParamType_STRING, .key = "desc",    .address = &SysIdentity.description,  .maxSize = SysIdentity.description.MAX_SIZE  },

      /*-----------------------------------------------------------------------
      Read/Write Parameters
      -----------------------------------------------------------------------*/
      ParameterNode{ .id = ParamId_PARAM_SERIAL_NUMBER,       .type = ParamType_STRING, .key = "ser_num",        .address = &SysIdentity.serialNumber,    .maxSize = SysIdentity.serialNumber.MAX_SIZE     },
      ParameterNode{ .id = ParamId_PARAM_DISK_UPDATE_RATE_MS, .type = ParamType_UINT32, .key = "dsk_updt",       .address = &SysConfig.diskUpdateRateMs,  .maxSize = sizeof( SysConfig.diskUpdateRateMs )  },
      ParameterNode{ .id = ParamId_PARAM_ACTIVITY_LED_SCALER, .type = ParamType_FLOAT,  .key = "actv_led_scale", .address = &SysConfig.activityLedScaler, .maxSize = sizeof( SysConfig.activityLedScaler ) },
      ParameterNode{ .id = ParamId_PARAM_BOOT_MODE,           .type = ParamType_UINT8,  .key = "boot_mode",      .address = &SysInfo.bootMode,            .maxSize = sizeof( SysInfo.bootMode )            },
      /***** Add new entries above here *****/
      /* clang-format on */
    };
  }    // namespace Internal

  static const ParameterList s_param_info = Internal::_compile_time_sort( Internal::_unsorted_parameters );

  static StaticJsonDocument<JSON_FILE_SIZE_MAX> s_json_cache;
  static Chimera::Thread::RecursiveMutex        s_json_lock;
  static etl::array<char, 64>                   s_fmt_buffer;
  static bool                                   s_json_pend_changes;
  static bool                                   s_json_is_synced;
  static uint32_t                               s_json_last_crc;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Loads the default values for all parameters
   */
  static void load_defaults()
  {
    SysCalibration.setDefaults();
    SysConfig.setDefaults();
    SysControl.setDefaults();
    SysIdentity.setDefaults();
    SysInfo.setDefaults();
  }


  /**
   * @brief Deserializes the JSON cache into the parameter registry
   */
  static void deserialize_disk_cache()
  {
    for ( size_t idx = 0; idx < s_param_info.size(); idx++ )
    {
      /*-----------------------------------------------------------------------
      Validate the key exists in the document
      -----------------------------------------------------------------------*/
      const ParameterNode *node = &s_param_info[ idx ];
      if ( !s_json_cache.containsKey( node->key ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Decode according to the marked format
      -----------------------------------------------------------------------*/
      switch ( node->type )
      {
        case ParamType_UINT8: {
          uint8_t *val = static_cast<uint8_t *>( node->address );
          *val         = s_json_cache[ node->key ].as<uint8_t>();
          break;
        }

        case ParamType_UINT16: {
          uint16_t *val = static_cast<uint16_t *>( node->address );
          *val          = s_json_cache[ node->key ].as<uint16_t>();
          break;
        }

        case ParamType_UINT32: {
          uint32_t *val = static_cast<uint32_t *>( node->address );
          *val          = s_json_cache[ node->key ].as<uint32_t>();
          break;
        }

        case ParamType_FLOAT: {
          float *val = static_cast<float *>( node->address );
          *val       = s_json_cache[ node->key ].as<float>();
          break;
        }

        case ParamType_DOUBLE: {
          double *val = static_cast<double *>( node->address );
          *val        = s_json_cache[ node->key ].as<double>();
          break;
        }

        case ParamType_STRING: {
          auto *val = static_cast<etl::istring *>( node->address );
          val->assign( s_json_cache[ node->key ].as<const char *>() );
          break;
        }

        default: {
          LOG_WARN( "Unsupported format specifier for key: %s", node->key );
          break;
        }
      }
    }
  }


  /**
   * @brief Get the CRC of the JSON cache
   * @return uint32_t
   */
  static uint32_t get_json_crc()
  {
    /*-------------------------------------------------------------------------
    Take a snapshot of the current JSON cache
    -------------------------------------------------------------------------*/
    char file_data[ JSON_FILE_SIZE_MAX ];
    memset( file_data, 0, sizeof( file_data ) );
    const size_t serialized_size = serializeJson( s_json_cache, file_data, sizeof( file_data ) );

    /*-------------------------------------------------------------------------
    Calculate the CRC of the snapshot
    -------------------------------------------------------------------------*/
    etl::crc32 crc;
    size_t     idx = 0;
    while ( idx < serialized_size )
    {
      crc.add( file_data[ idx++ ] );
    }

    return crc.value();
  }


  /**
   * @brief Finds a parameter node by its ID
   *
   * @param id    The ID of the parameter to find
   * @return const ParameterNode*
   */
  static const ParameterNode *find_parameter( const int id )
  {
    auto iterator = etl::find_if( s_param_info.begin(), s_param_info.end(),
                                  [ id ]( const ParameterNode &node ) { return static_cast<int>( node.id ) == id; } );

    if ( iterator != s_param_info.end() )
    {
      return iterator;
    }
    else
    {
      return nullptr;
    }
  }


  /**
   * @brief Finds a parameter node by its key
   *
   * @param key   The key of the parameter to find
   * @return const ParameterNode*
   */
  static const ParameterNode *find_parameter( const etl::string_view &key )
  {
    auto iterator = etl::find_if( s_param_info.begin(), s_param_info.end(),
                                  [ key ]( const ParameterNode &node ) { return key.compare( node.key ) == 0; } );

    if ( iterator != s_param_info.end() )
    {
      return iterator;
    }
    else
    {
      return nullptr;
    }
  }


  /**
   * @brief Prunes the JSON cache of any keys that are no longer valid
   */
  static void prune_json_document()
  {
    JsonObject root = s_json_cache.as<JsonObject>();
    for ( JsonPair kv : root )
    {
      const ParameterNode *node = find_parameter( kv.key().c_str() );
      if ( !node )
      {
        s_json_cache.remove( kv.key().c_str() );
      }
    }

    s_json_cache.garbageCollect();
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool loadDisk()
  {
    Chimera::Thread::LockGuard _lck( s_json_lock );

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    FS::FileId fd       = -1;
    s_json_pend_changes = false;

    /*-------------------------------------------------------------------------
    Always load the default configuration as a fallback
    -------------------------------------------------------------------------*/
    load_defaults();

    /*-------------------------------------------------------------------------
    Open the file
    -------------------------------------------------------------------------*/
    LOG_INFO( "Loading configuration from disk..." );
    if ( 0 != FS::fopen( Internal::SystemConfigFile.cbegin(), FILE_FLAGS, fd ) )
    {
      LOG_ERROR( "Failed to open configuration file: %s", Internal::SystemConfigFile.cbegin() );
      return false;
    }

    /*-------------------------------------------------------------------------
    Check the size of the file and load default parameters if required
    -------------------------------------------------------------------------*/
    size_t file_size = FS::fsize( fd );

    if ( file_size > JSON_FILE_SIZE_MAX )
    {
      LOG_ERROR( "Not enough memory to load file of size %d bytes", file_size );
      FS::fclose( fd );
      return false;
    }
    else if ( file_size == 0 )
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
    const char          *copy_ptr = file_data;
    DeserializationError error    = deserializeJson( s_json_cache, copy_ptr );
    if ( error )
    {
      s_json_last_crc = 0;
      LOG_ERROR( "Failed json decode: %s", error.c_str() );
      return false;
    }

    /*-------------------------------------------------------------------------
    Update the CRC and deserialize the disk cache into the parameter nodes
    -------------------------------------------------------------------------*/
    s_json_last_crc = get_json_crc();
    deserialize_disk_cache();

    /*-------------------------------------------------------------------------
    Once the disk cache is loaded, update the disk cache with new defaults
    -------------------------------------------------------------------------*/
    updateDiskCache();

    /*-------------------------------------------------------------------------
    Prune the JSON document of any keys that are no longer valid
    -------------------------------------------------------------------------*/
    prune_json_document();

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
    if ( s_json_pend_changes )
    {
      Chimera::Thread::LockGuard _lck( s_json_lock );
      const uint32_t             new_crc = get_json_crc();

      if ( new_crc != s_json_last_crc )
      {
        LOG_INFO( "Detected changes to configuration. Syncing with disk." );
        if ( flushDisk() )
        {
          s_json_last_crc  = new_crc;
          s_json_is_synced = true;
        }
        else
        {
          LOG_ERROR( "Failed to sync configuration with disk." );
        }
      }
      else
      {
        s_json_is_synced = true;
      }
    }
  }


  bool syncedToDisk()
  {
    return s_json_is_synced;
  }


  bool updateDiskCache( const ParamId param )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    const ParameterNode *node = find_parameter( param );
    if ( !node )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Serialize the data to strings for storage
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( s_json_lock );

    s_fmt_buffer.fill( 0 );
    switch ( node->type )
    {
      case ParamType_BOOL: {
        RT_DBG_ASSERT( node->maxSize == sizeof( bool ) );
        auto val = reinterpret_cast<bool *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_BOOL, *( val ) );
        break;
      }

      case ParamType_UINT8: {
        RT_DBG_ASSERT( node->maxSize == sizeof( uint8_t ) );
        auto val = reinterpret_cast<uint8_t *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_UINT8, *( val ) );
        break;
      }

      case ParamType_UINT16: {
        RT_DBG_ASSERT( node->maxSize == sizeof( uint16_t ) );
        auto val = reinterpret_cast<uint16_t *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_UINT16, *( val ) );
        break;
      }

      case ParamType_UINT32: {
        RT_DBG_ASSERT( node->maxSize == sizeof( uint32_t ) );
        auto val = reinterpret_cast<uint32_t *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_UINT32, *( val ) );
        break;
      }

      case ParamType_FLOAT: {
        RT_DBG_ASSERT( node->maxSize == sizeof( float ) );
        auto val = reinterpret_cast<float *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_FLOAT, *( val ) );
        break;
      }

      case ParamType_DOUBLE: {
        RT_DBG_ASSERT( node->maxSize == sizeof( double ) );
        auto val = reinterpret_cast<double *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_DOUBLE, *( val ) );
        break;
      }

      case ParamType_STRING: {
        auto val = reinterpret_cast<etl::istring *>( node->address );
        npf_snprintf( s_fmt_buffer.data(), s_fmt_buffer.size(), FMT_STRING, val->cbegin() );
        break;
      }

      default: {
        /* Missing format specifier */
        RT_DBG_ASSERT( false );
        return false;
      }
    }

    /*-------------------------------------------------------------------------
    Update the JSON document cache with the value
    -------------------------------------------------------------------------*/
    s_json_cache[ node->key ] = s_fmt_buffer.data();
    s_json_pend_changes       = true;
    s_json_is_synced          = false;
    return true;
  }


  bool updateDiskCache()
  {
    Chimera::Thread::LockGuard _lck( s_json_lock );

    bool sticky_result = true;
    for ( size_t idx = 0; idx < s_param_info.size(); idx++ )
    {
      auto &node = s_param_info[ idx ];
      sticky_result &= updateDiskCache( node.id );
    }

    return sticky_result;
  }


  bool copyFromCache( const ParamId param, void *const dest, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    const ParameterNode *node = find_parameter( param );
    if ( !node )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Copy the data from the cache
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( s_json_lock );
    if ( s_json_cache.containsKey( node->key ) )
    {
      const auto proxy = s_json_cache[ node->key ];
      const auto data  = proxy.as<std::string_view>();
      memcpy( dest, data.data(), std::min( size, data.size() ) );
      return true;
    }
    else
    {
      return false;
    }
  }


  bool copyToCache( const ParamId param, const void *const src, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    const ParameterNode *node = find_parameter( param );
    if ( !node )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Safely copy data to the working cache, then update the JSON document cache
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( s_json_lock );
    auto                       mask = Chimera::System::disableInterrupts();
    {
      if ( size > node->maxSize )
      {
        Chimera::System::enableInterrupts( mask );
        LOG_ERROR( "Parameter size mismatch. Max: %u, Size: %u", size, node->maxSize );
        return false;
      }

      memcpy( node->address, src, size );
    }
    Chimera::System::enableInterrupts( mask );
    return updateDiskCache( param );
  }


  ParamType getParamType( const ParamId param )
  {
    const ParameterNode *node = find_parameter( param );
    if ( node )
    {
      return node->type;
    }
    else
    {
      return ParamType::ParamType_UNKNOWN;
    }
  }


  bool paramExists( const int param )
  {
    return find_parameter( param ) != nullptr;
  }
}    // namespace Orbit::Data
