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
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/control/subroutines/declarations.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/control/subroutines/rotor_detector.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctl.hpp>
#include <src/monitor/orbit_monitors.hpp>

namespace Orbit::Tasks::CTL
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Control::Subroutine::IdleSubroutine s_idle_routine;
  static Control::Subroutine::RotorDetector  s_rotor_pos_detector_routine;

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
          Control::FOC::sendSystemEvent( Control::EventId::ARM );
          break;

        case TASK_MSG_CTRL_ENGAGE:
          Control::FOC::sendSystemEvent( Control::EventId::ENGAGE );
          break;

        case TASK_MSG_CTRL_DISABLE:
          Control::FOC::sendSystemEvent( Control::EventId::DISABLE );
          break;

        case TASK_MSG_CTRL_FAULT:
          Control::FOC::sendSystemEvent( Control::EventId::FAULT );
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
    Bind the motor control routines to define runtime behavior
    -------------------------------------------------------------------------*/
    Control::Subroutine::bind( Control::Subroutine::Routine::IDLE, &s_idle_routine );
    Control::Subroutine::bind( Control::Subroutine::Routine::ALIGNMENT_DETECTION, &s_rotor_pos_detector_routine );

    Control::Subroutine::switchRoutine( Control::Subroutine::Routine::IDLE );

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
      Process the motor control application layer
      -----------------------------------------------------------------------*/
      Control::Subroutine::process();

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::CTL
