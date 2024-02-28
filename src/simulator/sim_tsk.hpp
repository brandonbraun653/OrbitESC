/******************************************************************************
 *  File Name:
 *    sim_tsk.hpp
 *
 *  Description:
 *    Interface to the core simulator task
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TASK_SIMULATOR_HPP
#define ORBIT_ESC_TASK_SIMULATOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>


namespace Orbit::Tasks::SIM
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        PERIOD_MS = 5;
  static constexpr size_t                        STACK     = STACK_BYTES( 8192 );
  static constexpr std::string_view              NAME      = "sim";
  static constexpr Chimera::Thread::TaskPriority PRIORITY  = 5;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for SIM task
   *
   * @param arg   Unused
   */
  void SIMThread( void *arg );

}    // namespace Orbit::Tasks::SIM

#endif /* !ORBIT_ESC_TASK_SIMULATOR_HPP */
