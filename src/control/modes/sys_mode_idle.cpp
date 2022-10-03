/******************************************************************************
 *  File Name:
 *    sys_mode_idle.cpp
 *
 *  Description:
 *    Idle state
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_idle.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr bool DEBUG_MODULE = true;

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Idle::on_exit_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Exiting Idle state\r\n" );
  }


  etl::fsm_state_id_t Idle::on_enter_state()
  {
    /*-------------------------------------------------------------------------
    Disable the drive signals going to the motor
    -------------------------------------------------------------------------*/
    get_fsm_context().mTimerDriver.disableOutput();

    /*-------------------------------------------------------------------------
    Reset the motor controller
    -------------------------------------------------------------------------*/
    get_fsm_context().mState.emfObserver.clear();
    get_fsm_context().mState.speedEstimator.clear();
    get_fsm_context().mState.motorCtl.clear();

    LOG_TRACE_IF( DEBUG_MODULE, "Entered Idle state\r\n" );
    return ModeId::IDLE;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgEmergencyHalt &msg )
  {
    return this->No_State_Change;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgArm &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the ARMED state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgFault &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Idle::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
