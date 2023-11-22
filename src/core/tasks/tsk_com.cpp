/******************************************************************************
 *  File Name:
 *    tsk_com.cpp
 *
 *  Description:
 *    Communications task
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/utility>
#include <Chimera/thread>
#include <src/core/com/com_app_tx.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/runtime/can_runtime.hpp>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_com.hpp>

namespace Orbit::Tasks::COM
{
  std::array<char, 32> test_msg;

  static void msg_callback( Orbit::COM::Scheduler::Task *tsk )
  {
    test_msg.fill( 0 );
    npf_snprintf( test_msg.data(), test_msg.size(), "%d: Hello World!\r\n", Chimera::millis() );

    tsk->size = strlen( test_msg.data() );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void COMThread( void *arg ){

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize the communication busses
    -------------------------------------------------------------------------*/
    Orbit::CAN::initRuntime();
    Orbit::Serial::initRuntime();

    Orbit::COM::Scheduler::Task tsk;
    tsk.updater = msg_callback;
    tsk.data     = test_msg.data();
    tsk.endpoint = Orbit::COM::Scheduler::Endpoint::UART;
    tsk.period   = 5;
    tsk.priority = Orbit::COM::Scheduler::Priority::NORMAL;
    tsk.size     = strlen( test_msg.data() );
    tsk.ttl      = Orbit::COM::Scheduler::TTL_INFINITE;

    auto id = Orbit::COM::Scheduler::add( tsk );
    Orbit::COM::Scheduler::enable( id );

    Orbit::COM::initPeriodicData();

    /*-------------------------------------------------------------------------
    Run the thread
    -------------------------------------------------------------------------*/
    size_t last_wakeup = Chimera::millis();
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Process available work for each bus
      -----------------------------------------------------------------------*/
      Orbit::CAN::processCANBus();
      Orbit::Serial::processSerial();

      /*-----------------------------------------------------------------------
      Publish available data to the remote host
      -----------------------------------------------------------------------*/
      Orbit::COM::Scheduler::process();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( last_wakeup + PERIOD_MS );
      last_wakeup = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::COM
