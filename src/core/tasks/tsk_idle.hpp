/******************************************************************************
 *  File Name:
 *    tsk_idle.hpp
 *
 *  Description:
 *    Interface to the Idle task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TASK_IDLE_HPP
#define ORBIT_ESC_TASK_IDLE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>

namespace Orbit::Tasks::Idle
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t STACK                           = STACK_BYTES( 512 );
  static constexpr std::string_view NAME                  = "idle";
  static constexpr Chimera::Thread::TaskPriority PRIORITY = 0;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for idle task
   *
   * @param arg   Unused
   */
  void IdleThread( void *arg );

}  // namespace Orbit::Tasks::Idle

#endif  /* !ORBIT_ESC_TASK_IDLE_HPP */
