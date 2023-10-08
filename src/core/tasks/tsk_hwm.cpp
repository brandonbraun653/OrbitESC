/******************************************************************************
 *  File Name:
 *    tsk_hwm.cpp
 *
 *  Description:
 *    Hardware manager task
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/memory>
#include <Chimera/can>
#include <Chimera/thread>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_hwm.hpp>

namespace Orbit::Tasks::HWM
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void HWMThread( void *arg )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Run the HWM thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    TaskMsg tsk_msg     = TASK_MSG_NUM_OPTIONS;

    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Process any task messages that may have arrived
      -----------------------------------------------------------------------*/
      if ( this_thread::receiveTaskMsg( tsk_msg, TIMEOUT_DONT_WAIT ) )
      {
        switch( tsk_msg )
        {
          default:
            break;
        }
      }

      /*-----------------------------------------------------------------------
      Process hardware drivers
      -----------------------------------------------------------------------*/
      Orbit::LED::sendUpdate();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::HWM
