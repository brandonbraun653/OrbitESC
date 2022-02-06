/********************************************************************************
 *  File Name:
 *    version.hpp
 *
 *  Description:
 *    OrbitESC Version
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef ORBIT_VERSION_HPP
#define ORBIT_VERSION_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <string_view>

namespace Orbit
{
  /**
   *  CHANGELOG;
   *
   *  v0.0.0: Initial version
   */
  static constexpr std::string_view version = "0.0.0";

}    // namespace Orbit

#endif /* !ORBIT_VERSION_HPP */
