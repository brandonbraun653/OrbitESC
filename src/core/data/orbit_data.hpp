/******************************************************************************
 *  File Name:
 *    orbit_data.hpp
 *
 *  Description:
 *    Orbit data storage interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_HPP
#define ORBIT_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the data storage system
   *
   * @return bool
   */
  bool initialize();

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_HPP */
