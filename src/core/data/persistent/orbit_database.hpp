/******************************************************************************
 *  File Name:
 *    orbit_database.hpp
 *
 *  Description:
 *    Database interface for OrbitESC
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_DATABASE_HPP
#define ORBIT_ESC_DATABASE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

namespace Orbit::Data::Persistent
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initializes the persistent storage layer
   * @return void
   */
  void db_init();

  /**
   * @brief Writes a value to the database
   *
   * @param key     Key to write to
   * @param value   Value to write
   * @param len     Length of the value to write in bytes
   * @return size_t Number of bytes written
   */
  size_t db_write( const char *key, const void *value, const size_t len );

  /**
   * @brief Reads a value from the database
   *
   * @param key     Key to read from
   * @param value   Buffer to store the read value
   * @param len     Length of the data to read in bytes
   * @return size_t Number of bytes read
   */
  size_t db_read( const char *key, void *value, const size_t len );

}  // namespace Orbit::Data::Persistent

#endif  /* !ORBIT_ESC_DATABASE_HPP */
