/******************************************************************************
 *  File Name:
 *    com_scheduler.hpp
 *
 *  Description:
 *    Service for scheduling the transmission of messages on project IO. This
 *    module is intended to be used by the application layer to easily push
 *    data out the IO endpoints without having to dedicate a lot of time to
 *    managing the timing of the transmissions.
 *
 *    There are quite a large number of messages that need to be sent and
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_COM_SCHEDULER_HPP
#define ORBIT_ESC_COM_SCHEDULER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <src/core/com/com_types.hpp>


namespace Orbit::COM::Scheduler
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  using TaskId = int16_t;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief Maximum number of tasks that can be registered with the scheduler
   */
  static constexpr size_t MAX_TASKS = 16;

  /**
   * @brief An invalid task ID
   */
  static constexpr TaskId INVALID_TASK_ID = -1;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Options for setting the task's time to live
   */
  enum TTL_t : int16_t
  {
    TTL_INFINITE = -1,
    TTL_ONE_SHOT = 0,
    /* Anything greater than this is TTL in milliseconds */
  };

  /**
   * @brief Options for setting the task's priority
   */
  enum class Priority : uint8_t
  {
    LOWEST = 0,
    LOW,
    NORMAL,
    HIGH,
    HIGHEST,
    NUM_OPTIONS
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Core data structure representing a task to be run
   */
  struct Task
  {
    Endpoint endpoint;                 /**< Mask of which endpoint to write to */
    TaskId   uuid;                     /**< Registration ID of this task */
    Priority priority;                 /**< Assigns scheduling importance */
    uint16_t period;                   /**< How often to run the task in milliseconds */
    TTL_t    ttl;                      /**< How long to run the task in milliseconds */
    uint32_t nextRun;                  /**< Next time the task should run in milliseconds */
    void    *data;                     /**< Data to write. Must be persistent! (aka no stack memory) */
    uint32_t size;                     /**< Size of the data to write */
    void ( *callback )( Task *const ); /**< (Optional) Function to call when the task runs */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the scheduler service
   *
   * @return Chimera::Status_t
   */
  void initialize();

  /**
   * @brief Periodic processing of the scheduler service.
   *
   * This needs to be called at a regular interval to ensure the scheduler
   * is able to run tasks on time. The period at which this is called also
   * determines the resolution of the scheduler.
   *
   * @return void
   */
  void process();

  /**
   * @brief Registers a task with the scheduler.
   *
   * This will not enable the task for execution. It must be enabled manually.
   *
   * @param task      Task to register
   * @return TaskId   Assigned unique ID of the task, or INVALID_TASK_ID if failed
   */
  TaskId add( Task &task );

  /**
   * @brief Gets a reference to a task for inspection
   *
   * @param id    ID of the task to get
   * @return const Task*
   */
  const Task *get( const TaskId id );

  /**
   * @brief Removes a task from the scheduler
   *
   * @param id      ID of the task to remove
   * @return void
   */
  void remove( const TaskId id );

  /**
   * @brief Replaces/updates a pre-existing task with a new one.
   *
   * This could be used to change the priority, period, or data of a task.
   *
   * @param id      ID of the task to replace
   * @param task    New task to replace the old one with
   * @return void
   */
  void update( const TaskId id, Task &task );

  /**
   * @brief Enables a task for execution
   *
   * @param id    ID of the task to enable
   * @return void
   */
  void enable( const TaskId id );

  /**
   * @brief Disables a task from execution
   *
   * @param id    ID of the task to disable
   * @return void
   */
  void disable( const TaskId id );
}  // namespace

#endif  /* !ORBIT_ESC_COM_SCHEDULER_HPP */
