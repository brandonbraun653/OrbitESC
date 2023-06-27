/******************************************************************************
 *  File Name:
 *    tsk_hwm.hpp
 *
 *  Description:
 *    Interface to the Hardware Management task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TASK_HARDWARE_MANAGER_HPP
#define ORBIT_ESC_TASK_HARDWARE_MANAGER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>


namespace Orbit::Tasks::HWM
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        PERIOD_MS = 5;
  static constexpr size_t                        STACK     = STACK_BYTES( 4096 );
  static constexpr std::string_view              NAME      = "hwm";
  static constexpr Chimera::Thread::TaskPriority PRIORITY  = 5;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for HWM task
   *
   * @param arg   Unused
   */
  void HWMThread( void *arg );

}    // namespace Orbit::Tasks::HWM

#endif /* !ORBIT_ESC_TASK_HARDWARE_MANAGER_HPP */
