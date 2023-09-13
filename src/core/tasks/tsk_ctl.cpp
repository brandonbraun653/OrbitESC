/******************************************************************************
 *  File Name:
 *    tsk_sys_ctrl.cpp
 *
 *  Description:
 *    Control system task
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <Chimera/thread>
#include <src/control/foc_driver.hpp>
#include <src/control/current_control.hpp>
#include <src/control/speed_control.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctl.hpp>
#include <src/monitor/orbit_monitors.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>

namespace Orbit::Tasks::CTL
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Process any task messages that may have arrived
   * @return void
   */
  static void process_task_messages()
  {
    using namespace Chimera::Thread;

    TaskMsg tsk_msg = TASK_MSG_NUM_OPTIONS;

    /*-------------------------------------------------------------------------
    Process any task messages that may have arrived
    -------------------------------------------------------------------------*/
    if ( this_thread::receiveTaskMsg( tsk_msg, TIMEOUT_DONT_WAIT ) )
    {
      switch( tsk_msg )
      {
        case TASK_MSG_CTRL_ARM:

          if( Orbit::Instrumentation::getSupplyVoltage() > 10.0f )
          {
            Control::Field::powerUp();
            Control::Speed::powerUp();
          }

          // todo FILL THIS OUT
          // Control::FOCDriver.sendSystemEvent( Control::EventId::ARM );
          break;

        case TASK_MSG_CTRL_DISARM:
          break;

        case TASK_MSG_CTRL_ENGAGE:
          break;

        case TASK_MSG_CTRL_DISENGAGE:
          break;

        case TASK_MSG_CTRL_FAULT:

          break;

        default:
          break;
      }
    }
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void CTLThread( void *arg )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Run the CTL thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Process any task messages that may have arrived
      -----------------------------------------------------------------------*/
      process_task_messages();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::CTL
