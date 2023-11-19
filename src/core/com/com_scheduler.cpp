/******************************************************************************
 *  File Name:
 *    com_scheduler.cpp
 *
 *  Description:
 *    Implementation of the scheduler service
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/can>
#include <Chimera/serial>
#include <Chimera/thread>
#include <etl/array.h>
#include <etl/list.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/monitor/orbit_metrics.hpp>


namespace Orbit::COM::Scheduler
{
  /*---------------------------------------------------------------------------
  Private Data
  ---------------------------------------------------------------------------*/
  static etl::array<Task, MAX_TASKS>     sTaskList; /**< Backing memory for tasks */
  static etl::list<Task*, MAX_TASKS>     sRunQueue; /**< All tasks scheduled to run */
  static Chimera::Thread::RecursiveMutex sLock;     /**< Mutex for thread safety */
  static TaskId                          sNextId;   /**< Next task ID to assign */
  static Chimera::Serial::Driver_rPtr    sSerial;   /**< UART endpoint */
  static Chimera::CAN::Driver_rPtr       sCAN;      /**< CAN bus endpoint */
  static Orbit::Serial::USBSerial *      sUSB;      /**< USB endpoint */

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Sorts the task list by priority and next run time.
   *
   * The goal of this function is to order the task list in such a way that
   * the tasks which need to run the soonest are at the front of the list and
   * ordered by priority. This allows the scheduler to only require checking the
   * head of the list to determine which task needs to run next.
   *
   * @param lhs   Left hand side task to compare
   * @param rhs   Right hand side task to compare
   * @return int
   */
  static int task_sort_compare( const Task *const lhs, const Task *const rhs )
  {
    /*-------------------------------------------------------------------------
    Sort by next run time first
    -------------------------------------------------------------------------*/
    if( lhs->nextRun != rhs->nextRun )
    {
      return ( lhs->nextRun < rhs->nextRun ) ? -1 : 1;
    }

    /*-------------------------------------------------------------------------
    Sort by priority second within all tasks that have the same next run time
    -------------------------------------------------------------------------*/
    return ( lhs->priority <= rhs->priority ) ? -1 : 1;
  }


  /**
   * @brief Gets the address of the next free task entry in the memory backing store
   * @return Task*  Pointer to the next free task entry, nullptr if none are available
   */
  static Task *next_free_task()
  {
    for( auto &task : sTaskList )
    {
      if( task.uuid == INVALID_TASK_ID )
      {
        return &task;
      }
    }

    return nullptr;
  }


  /**
   * @brief Finds a task in the memory backing store
   *
   * @param id  Task ID to search for
   * @return Task*  Pointer to the task if found, nullptr otherwise
   */
  static Task *find_task( const TaskId id )
  {
    for( auto &task : sTaskList )
    {
      if( task.uuid == id )
      {
        return &task;
      }
    }

    return nullptr;
  }


  /**
   * @brief Checks to see if the task is already scheduled
   *
   * @param id    Task ID to check
   * @return bool True if the task is already scheduled, false otherwise
   */
  static bool task_scheduled( const TaskId id )
  {
    for( auto &task : sRunQueue )
    {
      if( task->uuid == id )
      {
        return true;
      }
    }

    return false;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize module memory
    -------------------------------------------------------------------------*/
    sTaskList.fill( { .uuid = -1 } );
    sRunQueue.clear();
    sLock.unlock();
    sNextId = 0;

    /*-------------------------------------------------------------------------
    Get a reference to all the hardware endpoints
    -------------------------------------------------------------------------*/
    sSerial = Chimera::Serial::getDriver( IO::USART::serialChannel );
    RT_HARD_ASSERT( sSerial );

    sCAN = Chimera::CAN::getDriver( IO::CAN::channel );
    RT_HARD_ASSERT( sCAN );

    sUSB = Orbit::Serial::getUSBSerialDriver();
    RT_HARD_ASSERT( sUSB );
  }


  void process()
  {
    Chimera::Thread::LockGuard lck( sLock );

    /*-------------------------------------------------------------------------
    Run as many tasks as are available for this processing period
    -------------------------------------------------------------------------*/
    const auto currentTick = Chimera::millis();

    for( auto &task : sRunQueue )
    {
      if( currentTick >= task->nextRun )
      {
        /*---------------------------------------------------------------------
        Schedule the next run time
        ---------------------------------------------------------------------*/
        task->nextRun += task->period;
        if( task->nextRun < currentTick )
        {
          task->nextRun = currentTick + task->period;
          LOG_WARN( "Com Scheduler: Task %d period skipped", task->uuid );
        }

        /*---------------------------------------------------------------------
        Push the data into the desired transmit endpoint
        ---------------------------------------------------------------------*/
        if( task->endpoint & Endpoint::UART )
        {
          const auto size = sSerial->write( task->data, task->size, Chimera::Thread::TIMEOUT_DONT_WAIT );
          LOG_WARN_IF( size != static_cast<int>( task->size ), "Com Scheduler: Task %d UART write failed", task->uuid );
          Monitor::putDataTXEvent();
        }

        if( task->endpoint & Endpoint::USB )
        {
          const auto size = sUSB->write( task->data, task->size, Chimera::Thread::TIMEOUT_DONT_WAIT );
          LOG_WARN_IF( size != static_cast<int>( task->size ), "Com Scheduler: Task %d USB write failed", task->uuid );
          Monitor::putDataTXEvent();
        }

        if( task->endpoint & Endpoint::CAN )
        {
          const auto status = sCAN->send( *reinterpret_cast<Chimera::CAN::BasicFrame *>( task->data ) );
          LOG_WARN_IF( status != Chimera::Status::OK, "Com Scheduler: Task %d CAN write failed", task->uuid );
          Monitor::putDataTXEvent();
        }
      }
      else
      {
        break;
      }
    }

    /*-------------------------------------------------------------------------
    Resort the task list to ensure the next task to run is at the front
    -------------------------------------------------------------------------*/
    sRunQueue.sort( task_sort_compare );
  }


  TaskId add( Task &task )
  {
    Chimera::Thread::LockGuard lck( sLock );

    /*-------------------------------------------------------------------------
    Any space left in the task list?
    -------------------------------------------------------------------------*/
    Task *newTask = next_free_task();
    if( !newTask )
    {
      return INVALID_TASK_ID;
    }

    /*-------------------------------------------------------------------------
    Success! Copy the task into the list.
    -------------------------------------------------------------------------*/
    *newTask = task;
    newTask->uuid = sNextId++;
    newTask->nextRun = Chimera::millis() + newTask->period;

    return newTask->uuid;
  }


  const Task *get( const TaskId id )
  {
    return find_task( id );
  }


  void remove( const TaskId id )
  {
    Chimera::Thread::LockGuard lck( sLock );

    /*-------------------------------------------------------------------------
    Find the task to remove, disable it, and clear it out
    -------------------------------------------------------------------------*/
    for( auto &task : sTaskList )
    {
      if( task.uuid == id )
      {
        sRunQueue.remove( &task );
        task = { .uuid = INVALID_TASK_ID };
      }
    }
  }


  void update( const TaskId id, Task &task )
  {
    Chimera::Thread::LockGuard lck( sLock );

    /*-------------------------------------------------------------------------
    Find the task to update
    -------------------------------------------------------------------------*/
    Task *oldTask = find_task( id );
    if( !oldTask )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Update the task, keeping the same ID
    -------------------------------------------------------------------------*/
    *oldTask = task;
    oldTask->uuid = id;
  }


  void enable( const TaskId id )
  {
    Chimera::Thread::LockGuard lck( sLock );

    /*-------------------------------------------------------------------------
    Schedule the task if it isn't already
    -------------------------------------------------------------------------*/
    Task *task = find_task( id );
    if( !task || task_scheduled( id ) )
    {
      return;
    }

    sRunQueue.push_back( task );
    sRunQueue.sort( task_sort_compare );
  }


  void disable( const TaskId id )
  {
    Chimera::Thread::LockGuard lck( sLock );

    /*-------------------------------------------------------------------------
    Remove the task from the run queue if it exists
    -------------------------------------------------------------------------*/
    for( auto &task : sTaskList )
    {
      if( task.uuid == id )
      {
        sRunQueue.remove( &task );
      }
    }
  }

}    // namespace Orbit::COM::Scheduler
