/******************************************************************************
 *  File Name:
 *    sys_mode_armed.cpp
 *
 *  Description:
 *    Armed state
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_armed.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr bool DEBUG_MODULE = true;

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Armed::on_exit_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Exiting ARMED state\r\n" );
  }

  etl::fsm_state_id_t Armed::on_enter_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Entered ARMED state\r\n" );
    return ModeId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgAlign &msg )
  {
    // /*-------------------------------------------------------------------------
    // Grab a reference to the controller and reset it
    // -------------------------------------------------------------------------*/
    // ParkControl *const pCtl = &get_fsm_context().mState.motorCtl.park;
    // pCtl->clear();

    // /*-------------------------------------------------------------------------
    // Prepare the Park controller to execute inside the ISR. Must be initialized
    // before leaving this function as the ISR will cue off the state change.
    // -------------------------------------------------------------------------*/
    // pCtl->activeComState      = 0; // All outputs off
    // pCtl->lastUpdate_ms       = Chimera::millis();
    // pCtl->startTime_ms        = Chimera::millis();
    // pCtl->alignTime_ms        = 3000;
    // pCtl->modulation_dt_ms    = 150;
    // pCtl->outputEnabled       = false;
    // pCtl->phaseDutyCycle[ 0 ] = 15.0f;
    // pCtl->phaseDutyCycle[ 1 ] = 15.0f;
    // pCtl->phaseDutyCycle[ 2 ] = 15.0f;

    // /*-------------------------------------------------------------------------
    // Prime the power stage to be active but all outputs OFF
    // -------------------------------------------------------------------------*/
    // get_fsm_context().mTimerDriver.disableOutput();
    // get_fsm_context().mTimerDriver.setForwardCommState( 0 );
    // get_fsm_context().mTimerDriver.enableOutput();

    return ModeId::ENGAGED_PARK;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgEmergencyHalt &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgDisarm &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the IDLE state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::IDLE;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgFault &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }

  etl::fsm_state_id_t Armed::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
