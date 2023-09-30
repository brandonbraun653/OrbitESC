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
  static fdb_err_t s_kv_db_ready;

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
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Creates the backing file used for the database.
   *
   * FlashDB expects the backing memory to exist and be fully accessible, so
   * we need to create a file that is large enough to hold the entire database.
   */
  static void db_create_backing_file();

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
    uint32_t sectorSize  = 512;
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
    s_kv_db_ready = fdb_kvdb_init( &s_kv_db, "orbit_db", "fdb_kv_db1", &default_table, NULL );
    LOG_ERROR_IF( s_kv_db_ready != FDB_NO_ERR, "Failed to initialize FlashDB" );
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
    /*-------------------------------------------------------------------------
    Look up the data in the database
    -------------------------------------------------------------------------*/
    fdb_blob  blob;
    fdb_kv_get_blob( &s_kv_db, key, &blob );

    /*-------------------------------------------------------------------------
    Copy as much data as we can. Leave it up to the user to decide if reading
    less than the requested amount is an error or not.
    -------------------------------------------------------------------------*/
    const size_t read_size = ( len < blob.saved.len ) ? len : blob.saved.len;
    memcpy( value, blob.buf, read_size );

    return read_size;
  }


  /*---------------------------------------------------------------------------
  Static Function Definitions
  ---------------------------------------------------------------------------*/

  static void db_create_backing_file()
  {
    FS::FileId      fd      = -1;
    FS::AccessFlags rdflags = ( FS::AccessFlags::O_RDONLY | FS::AccessFlags::O_CREAT );

    /*-------------------------------------------------------------------------
    Ensure the file exists
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( 0 == FS::fopen( SystemConfigFile.cbegin(), rdflags, fd ) );
    RT_HARD_ASSERT( 0 == FS::fclose( fd ) );

    /*-------------------------------------------------------------------------
    Pad the file out to the desired size
    -------------------------------------------------------------------------*/
    if( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_RDWR, fd ) )
    {
      /*-----------------------------------------------------------------------
      Figure out the current size of the file
      -----------------------------------------------------------------------*/
      FS::fseek( fd, 0, FS::WhenceFlags::F_SEEK_END );
      const size_t  DatabaseSize  = FS::ftell( fd );
      const size_t  PaddingSize   = DB_MAX_SIZE - DatabaseSize;
      const uint8_t padding[ 64 ] = { 0xFF };

      if( PaddingSize < DB_MAX_SIZE )
      {
        /*---------------------------------------------------------------------
        Write chunk sized padding
        ---------------------------------------------------------------------*/
        size_t i;
        for ( i = 0; ( i * sizeof( padding ) ) < PaddingSize; i++ )
        {
          FS::fwrite( padding, 1, sizeof( padding ), fd );
          FS::fflush( fd );
        }

        /*---------------------------------------------------------------------
        Write the last (possibly) partial chunk
        ---------------------------------------------------------------------*/
        const size_t partial_write = ( PaddingSize - ( i * sizeof( padding ) ) );
        FS::fwrite( padding, 1, partial_write, fd );
        FS::fflush( fd );
      }

      FS::fclose( fd );
    }

    /*-------------------------------------------------------------------------
    Ensure the file is now at the desired size
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_RDONLY, fd ) );
    FS::fseek( fd, 0, FS::WhenceFlags::F_SEEK_END );
    RT_HARD_ASSERT( DB_MAX_SIZE <= FS::ftell( fd ) );
    FS::fclose( fd );
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

  if ( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_RDWR, fd ) )
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

  if ( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_RDWR, fd ) )
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


  if ( 0 == FS::fopen( SystemConfigFile.cbegin(), FS::AccessFlags::O_RDWR, fd ) )
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
