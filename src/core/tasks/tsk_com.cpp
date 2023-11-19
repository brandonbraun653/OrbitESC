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
#include <Chimera/thread>
#include <src/core/com/com_app_tx.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/runtime/can_runtime.hpp>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_com.hpp>

namespace Orbit::Tasks::COM
{
  const char test_msg[] = "Hello World!\r\n";

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
    tsk.ttl = Orbit::COM::Scheduler::TTL_INFINITE;
    tsk.priority = Orbit::COM::Scheduler::Priority::NORMAL;
    tsk.callback = nullptr;
    tsk.period = 1000;
    tsk.endpoint = Orbit::COM::Scheduler::Endpoint::UART;
    tsk.data = (void*)test_msg;
    tsk.size = sizeof(test_msg);

    auto id = Orbit::COM::Scheduler::add( tsk );
    Orbit::COM::Scheduler::enable( id );

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
      // Orbit::Com::publishPhaseCurrents();
      // Orbit::Com::publishPhaseVoltages();
      // Orbit::Com::publishStateEstimates();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( last_wakeup + PERIOD_MS );
      last_wakeup = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::COM
