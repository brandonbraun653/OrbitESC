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
#include <src/core/data/persistent/orbit_database.hpp>
#include <src/core/data/orbit_data_params.hpp>

#include <flashdb.h>


namespace Orbit::Data::Persistent
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr ParameterList ParamInfo = ParamSorter( Internal::_unsorted_parameters );

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief The FlashDB Key-Value database instance
   */
  static fdb_kvdb s_kv_db;
  static fdb_err_t s_kv_db_ready;

  /**
   * @brief Maps the memory backing for each parameter.
   *
   * The FlashDB library requires some kind of storage to act as the cache for
   * every key. We already have that information in the parameter list, so this
   * just maps the two together at the cost of a little extra memory.
   */
  static fdb_default_kv_node s_dflt_kv_table[ ParamInfo.size() ];

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initDatabase()
  {
    /*-------------------------------------------------------------------------
    Initialize default KV table
    -------------------------------------------------------------------------*/
    for ( size_t idx = 0; idx < ParamInfo.size(); idx++ )
    {
      s_dflt_kv_table[ idx ].key       = ParamInfo[ idx ].key;
      s_dflt_kv_table[ idx ].value     = ParamInfo[ idx ].address;
      s_dflt_kv_table[ idx ].value_len = ParamInfo[ idx ].maxSize;
    }

    /*-------------------------------------------------------------------------
    Initialize the KV control block
    -------------------------------------------------------------------------*/
    uint32_t sectorSize = 512;
    uint32_t dbMaxSize  = 4 * 1024;
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
}    // namespace Orbit::Data::Persistent


/*-----------------------------------------------------------------------------
FlashDB File Access Functions
-----------------------------------------------------------------------------*/

extern "C" fdb_err_t _fdb_file_read( fdb_db_t db, uint32_t addr, void *buf, size_t size )
{
  return FDB_NO_ERR;
}


extern "C" fdb_err_t _fdb_file_write( fdb_db_t db, uint32_t addr, const void *buf, size_t size, bool sync )
{
  return FDB_NO_ERR;
}


extern "C" fdb_err_t _fdb_file_erase( fdb_db_t db, uint32_t addr, size_t size )
{
  return FDB_NO_ERR;
}
