/******************************************************************************
 *  File Name:
 *    sys_mode_ramp.cpp
 *
 *  Description:
 *    Armed state
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_ramp.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr bool DEBUG_MODULE = true;

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void EngagedRamp::on_exit_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Exiting RAMP state\r\n" );
  }

  etl::fsm_state_id_t EngagedRamp::on_enter_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Entering RAMP state\r\n" );
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgRun &msg )
  {
    /*-------------------------------------------------------------------------
    Prepare the run controller
    -------------------------------------------------------------------------*/

    /*-------------------------------------------------------------------------
    Stop the ramp controller
    -------------------------------------------------------------------------*/
    // return ModeId::ENGAGED_RUN;
    return this->No_State_Change; // TODO BMB: Remove this
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgEmergencyHalt &msg )
  {
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgDisengage &msg )
  {
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgFault &msg )
  {
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t EngagedRamp::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
