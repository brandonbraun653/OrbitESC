/******************************************************************************
 *  File Name:
 *    orbit_database.cpp
 *
 *  Description:
 *    Database interface for OrbitESC
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/filesystem>
#include <src/core/data/persistent/orbit_database.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/data/volatile/orbit_parameter_type.hpp>
#include <src/core/data/volatile/orbit_parameter_decl.hpp>

#include <flashdb.h>


namespace Orbit::Data::Persistent
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief The FlashDB Key-Value database instance
   */
  static fdb_kvdb  s_kv_db;
  static fdb_err_t s_kv_db_ready;

  /**
   * @brief Maps the memory backing for each parameter.
   *
   * The FlashDB library requires some kind of storage to act as the cache for
   * every key. We already have that information in the parameter list, so this
   * just maps the two together at the cost of a little extra memory.
   */
  static fdb_default_kv_node s_dflt_kv_table[ Param::NUM_PARAMS ];

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void db_init()
  {
    /*-------------------------------------------------------------------------
    Initialize default KV table
    -------------------------------------------------------------------------*/
    size_t idx = 0;
    for ( const Param::Node &node : Param::list() )
    {
      RT_DBG_ASSERT( idx < ARRAY_COUNT( s_dflt_kv_table ) );

      s_dflt_kv_table[ idx ].key       = node.key;
      s_dflt_kv_table[ idx ].value     = node.address;
      s_dflt_kv_table[ idx ].value_len = node.maxSize;
      idx++;
    }

    /*-------------------------------------------------------------------------
    Initialize the KV control block
    -------------------------------------------------------------------------*/
    uint32_t sectorSize  = 512;
    uint32_t dbMaxSize   = 4 * 1024;
    bool     useFileMode = true;

    s_kv_db = { 0 };
    fdb_kvdb_control( &s_kv_db, FDB_KVDB_CTRL_SET_SEC_SIZE, reinterpret_cast<void *>( &sectorSize ) );
    fdb_kvdb_control( &s_kv_db, FDB_KVDB_CTRL_SET_MAX_SIZE, reinterpret_cast<void *>( &dbMaxSize ) );
    fdb_kvdb_control( &s_kv_db, FDB_KVDB_CTRL_SET_FILE_MODE, reinterpret_cast<void *>( &useFileMode ) );


    fdb_default_kv default_table;
    default_table.kvs = s_dflt_kv_table;
    default_table.num = ARRAY_COUNT( s_dflt_kv_table );

    /*-------------------------------------------------------------------------
    Initialize the KV database
    -------------------------------------------------------------------------*/
    s_kv_db_ready = fdb_kvdb_init( &s_kv_db, "orbit_db", "fdb_kv_db1", &default_table, NULL );
    LOG_ERROR_IF( s_kv_db_ready != FDB_NO_ERR, "Failed to initialize FlashDB" );
  }


  size_t db_write( const char *key, const void *value, const size_t len )
  {
    return 0;
  }


  size_t db_read( const char *key, void *value, const size_t len )
  {
    return 0;
  }
}    // namespace Orbit::Data::Persistent


/*-----------------------------------------------------------------------------
FlashDB File Access Functions
-----------------------------------------------------------------------------*/
namespace FS = ::Aurora::FileSystem;

extern "C" fdb_err_t _fdb_file_read( fdb_db_t db, uint32_t addr, void *buf, size_t size )
{
  using namespace Orbit::Data;

  FS::FileId fd         = -1;
  size_t     bytes_read = 0;

  if ( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_RDONLY, fd ) )
  {
    if ( 0 == FS::fseek( fd, addr, FS::WhenceFlags::F_SEEK_SET ) )
    {
      bytes_read = FS::fread( buf, 1u, size, fd );
    }

    FS::fclose( fd );
  }

  return ( bytes_read == size ) ? FDB_NO_ERR : FDB_READ_ERR;
}


extern "C" fdb_err_t _fdb_file_write( fdb_db_t db, uint32_t addr, const void *buf, size_t size, bool sync )
{
  using namespace Orbit::Data;

  FS::FileId fd            = -1;
  size_t     bytes_written = 0;

  if ( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_WRONLY, fd ) )
  {
    if ( 0 == FS::fseek( fd, addr, FS::WhenceFlags::F_SEEK_SET ) )
    {
      bytes_written = FS::fwrite( buf, 1u, size, fd );

      if ( sync )
      {
        FS::fflush( fd );
      }
    }

    FS::fclose( fd );
  }

  return ( bytes_written == size ) ? FDB_NO_ERR : FDB_WRITE_ERR;
}


extern "C" fdb_err_t _fdb_file_erase( fdb_db_t db, uint32_t addr, size_t size )
{
  using namespace Orbit::Data;

  constexpr size_t BUF_SIZE     = 32;
  FS::FileId       fd           = -1;
  size_t           bytes_erased = 0;
  size_t           i            = 0;
  uint8_t          buf[ BUF_SIZE ];


  if ( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_WRONLY, fd ) )
  {
    if ( 0 == FS::fseek( fd, addr, FS::WhenceFlags::F_SEEK_SET ) )
    {
      /*-----------------------------------------------------------------------
      Write as many full buffers as possible
      -----------------------------------------------------------------------*/
      for ( i = 0; i * BUF_SIZE < size; i++ )
      {
        memset( buf, 0xFF, BUF_SIZE );
        bytes_erased += FS::fwrite( buf, 1u, BUF_SIZE, fd );
      }

      /*-----------------------------------------------------------------------
      Write the last (possibly) partial buffer
      -----------------------------------------------------------------------*/
      memset( buf, 0xFF, BUF_SIZE );
      bytes_erased += FS::fwrite( buf, 1u, ( size - ( i * BUF_SIZE ) ), fd );
      FS::fflush( fd );
    }

    FS::fclose( fd );
  }

  return ( bytes_erased == size ) ? FDB_NO_ERR : FDB_ERASE_ERR;
}
