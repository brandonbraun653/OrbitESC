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


namespace Orbit::Data::Persistent
{
  /*-----------------------------------------------------------------------------
  Public Functions
  -----------------------------------------------------------------------------*/
  /**
   * @brief Initializes the persistent storage layer
   * @return void
   */
  void initDatabase();


}  // namespace Orbit::Data::Persistent

#endif  /* !ORBIT_ESC_DATABASE_HPP */
