/******************************************************************************
 *  File Name:
 *    tasks.hpp
 *
 *  Description:
 *    Project task declaration
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TASKS_HPP
#define ORBIT_ESC_TASKS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/thread>


namespace Orbit::Tasks
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum PrjTaskId : uint8_t
  {
    TASK_IDLE,     /**< Background task */
    TASK_HWM,      /**< Hardware manager */
    TASK_HWM_DIO,  /**< Delayed IO HW manager */
    TASK_CTRL_SYS, /**< Control system task */

    TASK_NUM_OPTIONS,
    TASK_UNKNOWN
  };
  static_assert( TASK_IDLE == 0 );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Creates the system tasks and initializes thread resources
   *  @return void
   */
  void initialize();

  /**
   *  Helper function to instruct the current task to wait for
   *  an initialization message to be passed to it.
   *
   *  @return void
   */
  void waitInit();

  /**
   *  Gets the system thread identifier, assigned during boot.
   *
   *  @param[in]  task        Which task to look up
   *  @return Chimera::Thread::TaskId
   */
  Chimera::Thread::TaskId getTaskId( const PrjTaskId task );

}  // namespace Orbit::Tasks

#endif  /* !ORBIT_ESC_TASKS_HPP */
