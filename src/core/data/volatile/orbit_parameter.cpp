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
#include <algorithm>
#include <etl/algorithm.h>
#include <etl/string.h>
#include <etl/string_view.h>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/data/volatile/orbit_parameter_decl.hpp>
#include <src/core/data/orbit_data_defaults.hpp>

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
    std::sort( result.begin(), result.end(),
               []( const Node &a, const Node &b ) -> bool { return a.id < b.id; } );
    return result;
  }

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static ParameterList ParamInfo = ParamSorter( Internal::_unsorted_parameters );


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  const ParameterList& list()
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
    if ( node )
    {
      return node->type;
    }
    else
    {
      return ParamType::ParamType_UNKNOWN;
    }
  }


  Node *find( const ParamId id )
  {
    auto iterator = etl::find_if( ParamInfo.begin(), ParamInfo.end(),
                                  [ id ]( const Node &node ) { return static_cast<int>( node.id ) == id; } );

    if ( iterator != ParamInfo.end() )
    {
      return iterator;
    }
    else
    {
      return nullptr;
    }
  }


  Node *find( const etl::string_view &key )
  {
    auto iterator = etl::find_if( ParamInfo.begin(), ParamInfo.end(),
                                  [ key ]( const Node &node ) { return key.compare( node.key ) == 0; } );

    if ( iterator != ParamInfo.end() )
    {
      return iterator;
    }
    else
    {
      return nullptr;
    }
  }


  bool read( const ParamId param, void *const dest, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    const Node *node = find( param );
    if ( !node )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Copy the data from the cache
    -------------------------------------------------------------------------*/
    if ( node->maxSize >= size )
    {
      memcpy( dest, node->address, size );
      return true;
    }
    else
    {
      return false;
    }
  }


  bool write( const ParamId param, const void *const src, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Find the parameter in the list
    -------------------------------------------------------------------------*/
    Node *node = find( param );
    if ( !node )
    {
      return false;
    }

    /*-----------------------------------------------------------------------
    Validate the size and data before copying
    -----------------------------------------------------------------------*/
    if ( ( size > node->maxSize ) || ( node->validator && !node->validator( *node, src, size ) ) )
    {
      return false;
    }

    /*-----------------------------------------------------------------------
    Adjust the copy method depending on the underlying type. Strings need to
    be handled differently than the rest of the POD types.
    -----------------------------------------------------------------------*/
    switch ( node->type )
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

    node->dirty = true;
    return true;
  }


  bool flush()
  {
   // /*-------------------------------------------------------------------------
    // Flush the data to disk
    // -------------------------------------------------------------------------*/
    // LOG_INFO( "Storing configuration to disk..." );
    // FS::FileId fd       = -1;
    // s_json_pend_changes = false;

    // if ( 0 == FS::fopen( Internal::SystemConfigFile.cbegin(), FILE_FLAGS, fd ) )
    // {
    //   FS::frewind( fd );
    //   const size_t written = FS::fwrite( file_data, 1u, serialized_size, fd );
    //   FS::fclose( fd );
    //   LOG_ERROR_IF( written != serialized_size, "Failed to flush all bytes to disk. Act:%d, Exp:%d", written, serialized_size );
    //   return written == serialized_size;
    // }
    // else
    // {
    //   LOG_ERROR( "Disk flush failed. Cannot open file." );
    //   return false;
    // }

    return false;
  }


  bool load()
  {
    return false;
  }


  bool synchronized()
  {
    return false;
  }
}    // namespace Orbit::Data::Param
