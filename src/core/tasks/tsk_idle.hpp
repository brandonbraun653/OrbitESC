/******************************************************************************
 *  File Name:
 *    tsk_idle.hpp
 *
 *  Description:
 *    Interface to the Idle task
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TASK_IDLE_HPP
#define ORBIT_ESC_TASK_IDLE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>

namespace Orbit::Tasks::BKD
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        STACK    = STACK_BYTES( 8192 );
  static constexpr std::string_view              NAME     = "bkd";
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

}    // namespace Orbit::Tasks::BKD

#endif /* !ORBIT_ESC_TASK_IDLE_HPP */
