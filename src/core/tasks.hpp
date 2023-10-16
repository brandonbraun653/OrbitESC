/******************************************************************************
 *  File Name:
 *    tasks.hpp
 *
 *  Description:
 *    Project task declaration
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
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
    TASK_IDLE, /**< Background task */
    TASK_HWM,  /**< Hardware manager */
    TASK_DIO,  /**< Delayed IO HW manager */
    TASK_CTL,  /**< Control system task */
    TASK_COM,  /**< Networking/communications processing */
    TASK_USB,  /**< USB hardware stack */
    TASK_CDC,  /**< USB CDC driver */

    TASK_NUM_OPTIONS,
    TASK_UNKNOWN
  };
  static_assert( TASK_IDLE == 0 );

  enum PrjTaskMsg : uint32_t
  {
    __PRJ_FIRST_MSG = Chimera::Thread::TSK_MSG_PRJ_FIRST_MSG,

    /*-------------------------------------------------------------------------
    Delayed IO
    -------------------------------------------------------------------------*/
    TASK_MSG_PARAM_IO_EVENT,  /**< Handle incoming ParamIO messages */

    /*-------------------------------------------------------------------------
    System Control
    -------------------------------------------------------------------------*/
    TASK_MSG_CTRL_ARM,        /**< Transition from IDLE to ARM */
    TASK_MSG_CTRL_DISARM,     /**< Transition from ARM to IDLE */
    TASK_MSG_CTRL_ENGAGE,     /**< Transition from ARM to ENGAGED */
    TASK_MSG_CTRL_DISENGAGE,  /**< Transition from ENGAGED to ARM */
    TASK_MSG_CTRL_FAULT,      /**< Transition from ENGAGED to FAULT */

    TASK_MSG_NUM_OPTIONS
  };

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

}    // namespace Orbit::Tasks

#endif /* !ORBIT_ESC_TASKS_HPP */
