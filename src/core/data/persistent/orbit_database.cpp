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
#include <Aurora/memory>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/data/persistent/orbit_database.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/data/volatile/orbit_parameter_decl.hpp>
#include <src/core/data/volatile/orbit_parameter_type.hpp>

#include <flashdb.h>


namespace Orbit::Data::Persistent
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace FS = ::Aurora::FileSystem;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t DB_MAX_SIZE = 4 * 1024; /**< 512KB */

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief The FlashDB Key-Value database instance
   */
  static fdb_kvdb  s_kv_db;

  /**
   * @brief Maps the memory backing for each parameter.
   *
   * The FlashDB library requires some kind of storage to act as the cache for
   * every key. We already have that information in the parameter list, so this
   * just maps the two together at the cost of a little extra memory.
   */
  static fdb_default_kv_node s_dflt_kv_table[ Param::NUM_PARAMS ];

  /**
   * @brief The NOR flash memory driver for the database
   */
  static Aurora::Memory::Flash::NOR::Driver s_nor;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void db_init()
  {
    /*-------------------------------------------------------------------------
    Initialize the NOR flash driver
    -------------------------------------------------------------------------*/
    auto props = Aurora::Memory::Flash::NOR::getProperties( Aurora::Memory::Flash::NOR::Chip::AT25SF081 );

    Aurora::Memory::DeviceAttr attr;
    attr.eraseSize = props->blockSize;
    attr.readSize  = props->blockSize;
    attr.writeSize = props->blockSize;

    RT_HARD_ASSERT( true == s_nor.assignChipSelect( IO::SPI::norCSPort, IO::SPI::norCSPin ) );
    RT_HARD_ASSERT( true == s_nor.configure( Aurora::Memory::Flash::NOR::Chip::AT25SF081, IO::SPI::spiChannel ) );
    RT_HARD_ASSERT( Aurora::Memory::Status::ERR_OK == s_nor.open( &attr ) );

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
    uint32_t sectorSize  = 4096;
    uint32_t dbMaxSize   = DB_MAX_SIZE;
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
    auto ready = fdb_kvdb_init( &s_kv_db, "orbit_db", KVDB_PARTITION_NAME, &default_table, NULL );
    LOG_ERROR_IF( ready != FDB_NO_ERR, "Failed to initialize FlashDB" );
  }


  size_t db_write( const char *key, const void *value, const size_t len )
  {
    /*-------------------------------------------------------------------------
    Write the data to the database
    -------------------------------------------------------------------------*/
    fdb_blob  blob;
    fdb_err_t error = fdb_kv_set_blob( &s_kv_db, key, fdb_blob_make( &blob, value, len ) );

    return ( error == FDB_NO_ERR ) ? len : 0;
  }


  size_t db_read( const char *key, void *value, const size_t len )
  {
    fdb_blob blob;
    fdb_kv_get_blob( &s_kv_db, key, fdb_blob_make( &blob, value, len ) );

    return blob.saved.len;
  }
}    // namespace Orbit::Data::Persistent


/*-----------------------------------------------------------------------------
FlashDB Interface Layer
-----------------------------------------------------------------------------*/
namespace FS = ::Aurora::FileSystem;

#ifdef __cplusplus
extern "C"
{
#endif
  using namespace Orbit::Data::Persistent;

  static int _fdb_flash_init( void );
  static int _fdb_flash_read( long offset, uint8_t *buf, size_t size );
  static int _fdb_flash_write( long offset, const uint8_t *buf, size_t size );
  static int _fdb_flash_erase( long offset, size_t size );

  const fal_flash_dev fdb_nor_flash0 = {
    .name = NOR_FLASH_DEV_NAME,
    .addr = 0,
    .len  = 1024 * 1024,
    .blk_size = 4096,
    .ops = {
      .init  = _fdb_flash_init,
      .read  = _fdb_flash_read,
      .write = _fdb_flash_write,
      .erase = _fdb_flash_erase,
    },
    .write_gran = 1
  };

  static int _fdb_flash_init( void )
  {
    return 0;
  }


  static int _fdb_flash_read( long offset, uint8_t *buf, size_t size )
  {
    return ( s_nor.read( offset, buf, size ) == Aurora::Memory::Status::ERR_OK ) ? size : -1;
  }


  static int _fdb_flash_write( long offset, const uint8_t *buf, size_t size )
  {
    return ( s_nor.write( offset, buf, size ) == Aurora::Memory::Status::ERR_OK ) ? size : -1;
  }


  static int _fdb_flash_erase( long offset, size_t size )
  {
    return ( s_nor.erase( offset, size ) == Aurora::Memory::Status::ERR_OK ) ? size : -1;
  }

#ifdef __cplusplus
}
#endif
