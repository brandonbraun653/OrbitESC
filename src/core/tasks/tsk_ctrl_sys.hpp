/******************************************************************************
 *  File Name:
 *    tsk_ctrl_sys.hpp
 *
 *  Description:
 *    Interface to the control system task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TASK_CONTROL_SYSTEM_HPP
#define ORBIT_ESC_TASK_CONTROL_SYSTEM_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>


namespace Orbit::Tasks::CTRLSYS
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        PERIOD_MS = 10;
  static constexpr size_t                        STACK     = STACK_BYTES( 2048 );
  static constexpr std::string_view              NAME      = "ctrl_sys";
  static constexpr Chimera::Thread::TaskPriority PRIORITY  = 4;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for CTRLSYS task
   *
   * @param arg   Unused
   */
  void CTRLSYSThread( void *arg );

}    // namespace Orbit::Tasks::CTRLSYS

#endif /* !ORBIT_ESC_TASK_CONTROL_SYSTEM_HPP */
