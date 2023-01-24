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
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_com.hpp>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/runtime/can_runtime.hpp>


namespace Orbit::Tasks::COM
{
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

    /*-------------------------------------------------------------------------
    Run the thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Process available work for each bus
      -----------------------------------------------------------------------*/
      Orbit::CAN::processCANBus();
      Orbit::Serial::processSerial();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::COM