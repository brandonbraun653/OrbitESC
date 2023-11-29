/******************************************************************************
 *  File Name:
 *    orbit_parameter.hpp
 *
 *  Description:
 *    Parameter listings for the whole orbit data system
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_PARAMETERS_HPP
#define ORBIT_DATA_PARAMETERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <etl/string_view.h>
#include <src/core/com/proto/serial_interface.pb.h>
#include <src/core/data/volatile/orbit_parameter_decl.hpp>

namespace Orbit::Data::Param
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initializes the parameter system
   * @return void
   */
  void init();

  /**
   * @brief Gets the core list storing parameter information
   * @return const ParameterList&
   */
  const ParameterList &list();

  /**
   * @brief Get storage type of the parameter
   *
   * @param param   Which parameter to query
   * @return ParamType
   */
  ParamType type( const ParamId param );

  /**
   * @brief Checks if the given parameter ID exists
   *
   * @param param   Parameter ID to check
   * @return bool   True if the parameter exists, false if not
   */
  bool exists( const ParamId param );

  /**
   * @brief Finds a parameter node by its ID
   *
   * @param id    The ID of the parameter to find
   * @return Node*
   */
  const Node *find( const ParamId id );

  /**
   * @brief Finds a parameter node by its key
   *
   * @param key   The key of the parameter to find
   * @return Node*
   */
  const Node *find( const etl::string_view &key );

  /**
   * @brief Binary copy a parameter from cache into a given buffer
   *
   * @param param   Which parameter to copy
   * @param dest    Where to copy the data to
   * @param size    Size of the destination buffer
   * @return size_t How many bytes were read, or negative on error
   */
  ssize_t read( const ParamId param, void *const dest, const size_t size );

  /**
   * @brief Binary write a parameter from a given buffer into cache
   *
   * @param param   Which parameter to update
   * @param src     Where to copy the data from
   * @param size    How many bytes to copy
   * @return bool   True if success, false if not
   */
  bool write( const ParamId param, const void *const src, const size_t size );

  /**
   * @brief Flushes all dirty parameters to disk
   * @return bool
   */
  bool flush();

  /**
   * @brief Loads all parameters from disk
   * @return bool
   */
  bool load();

  /**
   * @brief Status flag indicating if all parameters are saved to disk
   * @return bool
   */
  bool synchronized();

}    // namespace Orbit::Data::Param

#endif /* !ORBIT_DATA_PARAMETERS_HPP */
