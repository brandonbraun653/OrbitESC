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
#include <array>
#include <algorithm>
#include <cstddef>

#include <src/core/data/volatile/orbit_parameter_decl.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr const char *FMT_UINT8  = "%u";
  static constexpr const char *FMT_UINT16 = FMT_UINT8;
  static constexpr const char *FMT_UINT32 = "%lu";
  static constexpr const char *FMT_FLOAT  = "%4.9f";
  static constexpr const char *FMT_DOUBLE = "%4.17f";
  static constexpr const char *FMT_STRING = "%s";
  static constexpr const char *FMT_BOOL   = "%d";




  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using ParameterList = std::array<ParameterNode, Internal::_unsorted_parameters.size()>;


  /*---------------------------------------------------------------------------
  Public Functions
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
                []( const ParameterNode &a, const ParameterNode &b ) -> bool { return a.id < b.id; } );
    return result;
  }

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_PARAMETERS_HPP */
