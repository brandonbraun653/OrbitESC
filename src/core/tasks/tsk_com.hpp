/******************************************************************************
 *  File Name:
 *    tsk_com.hpp
 *
 *  Description:
 *    Task for handling system communication busses
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TSK_COM_HPP
#define ORBIT_TSK_COM_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>

namespace Orbit::Tasks::COM
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        PERIOD_MS = 5;
  static constexpr size_t                        STACK     = STACK_BYTES( 12 * 1024 );
  static constexpr std::string_view              NAME      = "com";
  static constexpr Chimera::Thread::TaskPriority PRIORITY  = 4;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for COM task
   *
   * @param arg   Unused
   */
  void COMThread( void *arg );

}  // namespace Orbit::Tasks::COM

#endif  /* !ORBIT_TSK_COM_HPP */
