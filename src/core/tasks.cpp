/******************************************************************************
 *  File Name:
 *    tasks.cpp
 *
 *  Description:
 *    Project task support drivers
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/utility>
#include <Chimera/common>
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_com.hpp>
#include <src/core/tasks/tsk_ctl.hpp>
#include <src/core/tasks/tsk_dio.hpp>
#include <src/core/tasks/tsk_hwm.hpp>
#include <src/core/tasks/tsk_idle.hpp>
#include <src/core/tasks/tsk_usb.hpp>


namespace Orbit::Tasks
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Thread::TaskId s_thread_id[ TASK_NUM_OPTIONS ];
  static uint32_t                s_bkd_thread_stack[ STACK_BYTES( BKD::STACK ) ] __attribute__((section(".app_stack")));
  static uint32_t                s_hwm_thread_stack[ STACK_BYTES( HWM::STACK ) ] __attribute__((section(".app_stack")));
  static uint32_t                s_dio_thread_stack[ STACK_BYTES( DIO::STACK ) ] __attribute__((section(".app_stack")));
  static uint32_t                s_ctl_thread_stack[ STACK_BYTES( CTL::STACK ) ] __attribute__((section(".app_stack")));
  static uint32_t                s_com_thread_stack[ STACK_BYTES( COM::STACK ) ] __attribute__((section(".app_stack")));
  static uint32_t                s_usb_thread_stack[ STACK_BYTES( USB::STACK ) ] __attribute__((section(".app_stack")));

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void init_idle_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.name                                  = BKD::NAME.data();
    cfg.arg                                   = nullptr;
    cfg.function                              = BKD::IdleThread;
    cfg.priority                              = BKD::PRIORITY;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_bkd_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_bkd_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_bkd_thread_stack );

    tsk.create( cfg );
    s_thread_id[ TASK_IDLE ] = tsk.start();
  }


  static void init_hwm_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.name                                  = HWM::NAME.data();
    cfg.arg                                   = nullptr;
    cfg.function                              = HWM::HWMThread;
    cfg.priority                              = HWM::PRIORITY;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_hwm_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_hwm_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_hwm_thread_stack );

    tsk.create( cfg );
    s_thread_id[ TASK_HWM ] = tsk.start();
  }


  static void init_hwm_dio_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.name                                  = DIO::NAME.data();
    cfg.arg                                   = nullptr;
    cfg.function                              = DIO::DIOThread;
    cfg.priority                              = DIO::PRIORITY;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_dio_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_dio_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_dio_thread_stack );

    tsk.create( cfg );
    s_thread_id[ TASK_DIO ] = tsk.start();
  }


  static void init_ctrl_sys_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.name                                  = CTL::NAME.data();
    cfg.arg                                   = nullptr;
    cfg.function                              = CTL::CTLThread;
    cfg.priority                              = CTL::PRIORITY;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_ctl_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_ctl_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_ctl_thread_stack );

    tsk.create( cfg );
    s_thread_id[ TASK_CTL ] = tsk.start();
  }


  static void init_com_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.name                                  = COM::NAME.data();
    cfg.arg                                   = nullptr;
    cfg.function                              = COM::COMThread;
    cfg.priority                              = COM::PRIORITY;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_com_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_com_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_com_thread_stack );

    tsk.create( cfg );
    s_thread_id[ TASK_COM ] = tsk.start();
  }


  static void init_usb_task()
  {
    using namespace Chimera::Thread;

    TaskConfig cfg;
    Task       tsk;

    cfg.name                                  = USB::NAME.data();
    cfg.arg                                   = nullptr;
    cfg.function                              = USB::USBThread;
    cfg.priority                              = USB::PRIORITY;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_usb_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_usb_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_usb_thread_stack );

    tsk.create( cfg );
    s_thread_id[ TASK_USB ] = tsk.start();
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

    The order of task creation is important. The DIO task must be created first
    to handle RAM expensive operations like powering on the filesystem and
    loading in the configuration file. All other tasks are order agnostic.
    -------------------------------------------------------------------------*/
    init_hwm_dio_task();  /* This one must run first */
    init_idle_task();
    init_hwm_task();
    init_ctrl_sys_task();
    init_com_task();
    init_usb_task();
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
