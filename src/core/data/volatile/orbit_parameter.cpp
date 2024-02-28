/******************************************************************************
 *  File Name:
 *    orbit_parameter.cpp
 *
 *  Description:
 *    Parameters used for system configuration
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <algorithm>
#include <etl/algorithm.h>
#include <etl/forward_list.h>
#include <etl/string.h>
#include <etl/string_view.h>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/data/volatile/orbit_parameter_decl.hpp>
#include <src/core/data/persistent/orbit_database.hpp>

namespace Orbit::Data::Param
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Helper function to sort the parameter list by ID
   *
   * @param list Array of parameters to sort
   * @return constexpr ParameterList
   */
  static constexpr ParameterList ParamSorter( const ParameterList &list )
  {
    auto result = list;
    std::sort( result.begin(), result.end(), []( const Node &a, const Node &b ) -> bool { return a.id < b.id; } );
    return result;
  }

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief Descriptor for all supported parameters
   */
  static const ParameterList ParamInfo = ParamSorter( Internal::_unsorted_parameters );

  /**
   * @brief Maintenance list for parameters that have been modified and need flushing
   */
  static etl::forward_list<const Node *, Param::NUM_PARAMS> s_dirty_list;

  /**
   * @brief Protect write access to the parameter cache
   */
  static Chimera::Thread::RecursiveMutex s_write_lock;


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Reads the parameter from the database, accounting for the type.
   *
   * @param type  The type of parameter to read
   * @param key   The key to read from the database
   * @param value The value to read into
   * @param len   The length of the value to read
   * @return size_t   The number of bytes read
   */
  static size_t database_read( ParamType type, const char *key, void *const value, const size_t len )
  {
    char   string_buffer[ Param::MAX_STRING_SIZE ] = { 0 };
    size_t act_read_size                           = 0;
    size_t min_read_size                           = 0;

    switch( type )
    {
      case ParamType_STRING: {
        min_read_size = std::min( len, sizeof( string_buffer ) );
        act_read_size = Persistent::db_read( key, string_buffer, min_read_size );
        auto val      = reinterpret_cast<etl::istring *>( value );
        val->assign( string_buffer );
      }
      break;

      default:
        act_read_size = Persistent::db_read( key, value, len );
        break;
    }

    LOG_WARN_IF( act_read_size != len, "Database read size mismatch: %s, %d, %d", key, act_read_size, len );
    return act_read_size;
  }


  /**
   * @brief Writes the parameter to the database, accounting for the type.
   *
   * @param type The type of parameter to write
   * @param key  The key to write to the database
   * @param value  The value to write
   * @param len   The length of the value to write
   * @return size_t  The number of bytes written
   */
  static size_t database_write( ParamType type, const char *key, const void *value, const size_t len )
  {
    const void *write_ptr                               = nullptr;
    char        string_buffer[ Param::MAX_STRING_SIZE ] = { 0 };

    switch( type )
    {
      case ParamType_STRING: {
        auto val = reinterpret_cast<const etl::istring *>( value );
        memcpy( string_buffer, val->cbegin(), std::min( len, val->max_size() ) );
        write_ptr = string_buffer;
      }
      break;

      default:
        write_ptr = value;
        break;
    }

    const size_t act_write_size = Persistent::db_write( key, write_ptr, len );
    LOG_WARN_IF( act_write_size != len, "Database write size mismatch: %s, %d, %d", key, act_write_size, len );
    return act_write_size;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void init()
  {
    /*-------------------------------------------------------------------------
    Reset module memory cache
    -------------------------------------------------------------------------*/
    SysCalibration.clear();
    SysConfig.clear();
    SysControl.clear();
    SysIdentity.clear();
    SysInfo.clear();

    /*-------------------------------------------------------------------------
    Load all defaults
    -------------------------------------------------------------------------*/
    SysCalibration.setDefaults();
    SysConfig.setDefaults();
    SysControl.setDefaults();
    SysIdentity.setDefaults();
    SysInfo.setDefaults();

    /*-------------------------------------------------------------------------
    Intialize remaining module data
    -------------------------------------------------------------------------*/
    s_dirty_list.clear();
  }


  const ParameterList &list()
  {
    return ParamInfo;
  }


  bool exists( const ParamId param )
  {
    return find( param ) != nullptr;
  }


  ParamType type( const ParamId param )
  {
    const Node *node = find( param );
    if( node )
    {
      return node->type;
    }
    else
    {
      return ParamType::ParamType_UNKNOWN;
    }
  }


  const Node *find( const ParamId id )
  {
    auto iterator = etl::find_if( ParamInfo.begin(), ParamInfo.end(),
                                  [ id ]( const Node &node ) { return static_cast<int>( node.id ) == id; } );

    if( iterator != ParamInfo.end() )
    {
      return iterator;
    }
    else
    {
      return nullptr;
    }
  }


  const Node *find( const etl::string_view &key )
  {
    auto iterator = etl::find_if( ParamInfo.begin(), ParamInfo.end(),
                                  [ key ]( const Node &node ) { return key.compare( node.key ) == 0; } );

    if( iterator != ParamInfo.end() )
    {
      return iterator;
    }
    else
    {
      return nullptr;
    }
  }


  ssize_t read( const ParamId param, void *const dest, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    const Node *node = find( param );
    if( !node )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Copy the data from the cache
    -------------------------------------------------------------------------*/
    size_t read_size = std::min( size, static_cast<size_t>( node->maxSize ) );

    if( node->type == ParamType_STRING )
    {
      auto val  = reinterpret_cast<etl::istring *>( node->address );
      read_size = std::min( read_size, val->length() );
      val->copy( reinterpret_cast<char *>( dest ), read_size );
    }
    else
    {
      memcpy( dest, node->address, read_size );
    }

    return static_cast<ssize_t>( read_size );
  }


  bool write( const ParamId param, const void *const src, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    const Node *node = find( param );
    if( !node )
    {
      return false;
    }

    /*-----------------------------------------------------------------------
    Validate the size and data before copying
    -----------------------------------------------------------------------*/
    if( ( size > node->maxSize ) || ( node->validator && !node->validator( *node, src, size ) ) )
    {
      return false;
    }

    Chimera::Thread::LockGuard _lck( s_write_lock );

    /*-----------------------------------------------------------------------
    Ignore the update if no change has been made to the memory span
    -----------------------------------------------------------------------*/
    switch( node->type )
    {
      case ParamType_STRING: {
        auto val = reinterpret_cast<etl::istring *>( node->address );
        if( val->compare( reinterpret_cast<const char *>( src ) ) == 0 )
        {
          return true;
        }
      }
      break;

      default:
        /*---------------------------------------------------------------------
        If the data is the same, don't bother writing it to disk.
        ---------------------------------------------------------------------*/
        if( ( memcmp( node->address, src, size ) == 0 ) && ( node->address != src ) )
        {
          return true;
        }
        break;
    }

    /*-----------------------------------------------------------------------
    Adjust the copy method depending on the underlying type. Strings need to
    be handled differently than the rest of the POD types.
    -----------------------------------------------------------------------*/
    switch( node->type )
    {
      case ParamType_STRING: {
        auto val = reinterpret_cast<etl::istring *>( node->address );
        val->assign( reinterpret_cast<const char *>( src ), size );
      }
      break;

      default:
        memcpy( node->address, src, size );
        break;
    }

    /*-------------------------------------------------------------------------
    Add the node to the dirty list. Push to the front of the list using the
    assumption that more recently modified parameters are more likely to be
    modified again, decreasing search time.
    -------------------------------------------------------------------------*/
    auto iterator = etl::find( s_dirty_list.begin(), s_dirty_list.end(), node );
    if( iterator == s_dirty_list.end() )
    {
      s_dirty_list.push_front( node );
    }

    return true;
  }


  bool flush()
  {
    /*-------------------------------------------------------------------------
    Don't flush if nothing to do
    -------------------------------------------------------------------------*/
    if( synchronized() )
    {
      return true;
    }

    /*-------------------------------------------------------------------------
    Flush the dirty list to disk
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( s_write_lock );

    size_t max_attempts = 0;
    while( ( max_attempts < 5 ) && !s_dirty_list.empty() )
    {
      const Node *node = s_dirty_list.front();
      if( database_write( node->type, node->key, node->address, node->maxSize ) == node->maxSize )
      {
        s_dirty_list.pop_front();
        max_attempts = 0;
      }
      else
      {
        max_attempts++;
        LOG_ERROR( "Failed to write parameter to disk: %s", node->key );
      }
    }

    return synchronized();
  }


  bool load()
  {
    /*-------------------------------------------------------------------------
    Acquire the lock first to prevent any other threads from modifying the
    cache after the initial flush.
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( s_write_lock );

    /*-------------------------------------------------------------------------
    Ensure we've synchronized any dirty parameters
    -------------------------------------------------------------------------*/
    flush();

    /*-------------------------------------------------------------------------
    Pull each parameter from disk. If a parameter doesn't exist, program in the
    default value, which was set during the init() call.
    -------------------------------------------------------------------------*/
    for( const auto &node : ParamInfo )
    {
      const size_t read_size = database_read( node.type, node.key, node.address, node.maxSize );
      if( read_size == 0 )
      {
        LOG_INFO( "Missing parameter from disk: %s. Programming defaults.", node.key );
        database_write( node.type, node.key, node.address, node.maxSize );
      }

      LOG_WARN_IF( ( read_size != node.maxSize ) && ( read_size != 0 ),
                   "Size mismatch reading parameter %s from disk: %d != %d", node.key, read_size, node.maxSize );
    }

    /*-------------------------------------------------------------------------
    Cache is now clean
    -------------------------------------------------------------------------*/
    s_dirty_list.clear();
    return true;
  }


  bool synchronized()
  {
    return s_dirty_list.empty();
  }
}    // namespace Orbit::Data::Param
