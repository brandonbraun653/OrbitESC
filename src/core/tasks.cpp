/******************************************************************************
 *  File Name:
 *    tasks.cpp
 *
 *  Description:
 *    Project task support drivers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/utility>
#include <Chimera/common>
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_idle.hpp>


namespace Orbit::Tasks
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Thread::TaskId s_thread_id[ TASK_NUM_OPTIONS ];

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void init_idle_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.arg        = nullptr;
    cfg.function   = Idle::IdleThread;
    cfg.priority   = Idle::PRIORITY;
    cfg.stackWords = Idle::STACK;
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = Idle::NAME.data();

    tsk.create( cfg );
    s_thread_id[ TASK_IDLE ] = tsk.start();
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initialize()
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Initialize local memory
    -------------------------------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( s_thread_id ); x++ )
    {
      s_thread_id[ x ] = THREAD_ID_INVALID;
    }

    /*-------------------------------------------------------------------------
    Task Initialization
    -------------------------------------------------------------------------*/
    init_idle_task();
  }


  void waitInit()
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for task registration to complete. On the sim this happens so fast
    that trying to receive a task message will cause a fault. This is due to
    tasks starting at creation on PCs, but not on embedded.
    -------------------------------------------------------------------------*/
#if defined( CHIMERA_SIMULATOR )
    Chimera::delayMilliseconds( 50 );
#endif

    /*-------------------------------------------------------------------------
    Wait for the expected task message to arrive
    -------------------------------------------------------------------------*/
    TaskMsg msg = ITCMsg::TSK_MSG_NOP;
    while ( true )
    {
      if ( this_thread::receiveTaskMsg( msg, TIMEOUT_BLOCK ) && ( msg == ITCMsg::TSK_MSG_WAKEUP ) )
      {
        break;
      }
      else
      {
        this_thread::yield();
      }
    }
  }


  Chimera::Thread::TaskId getTaskId( const PrjTaskId task )
  {
    if ( !( task < TASK_NUM_OPTIONS ) )
    {
      return Chimera::Thread::THREAD_ID_INVALID;
    }

    return s_thread_id[ task ];
  }
}    // namespace Orbit::Tasks
