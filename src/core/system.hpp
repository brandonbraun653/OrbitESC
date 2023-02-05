/******************************************************************************
 *  File Name:
 *    system.hpp
 *
 *  Description:
 *    High level system behavioral interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SYSTEM_HPP
#define ORBIT_SYSTEM_HPP

namespace Orbit::System
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Safely shuts down the system
   */
  void doSafeShutdown();

}    // namespace Orbit::System

#endif /* !ORBIT_SYSTEM_HPP */
