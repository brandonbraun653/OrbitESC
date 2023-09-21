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
#include <src/core/data/persistent/orbit_database.hpp>


#include <flashdb.h>


namespace Orbit::Data::Persistent
{
  void initDatabase()
  {

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
