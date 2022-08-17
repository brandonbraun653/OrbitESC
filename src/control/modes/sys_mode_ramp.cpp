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
  State Class
  ---------------------------------------------------------------------------*/
  etl::fsm_state_id_t EngagedRamp::on_enter_state()
  {
    LOG_INFO( "Entering Ramp state\r\n" );
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgEmergencyHalt &msg )
  {
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgDisengage &msg )
  {
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgRun &msg )
  {
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedRamp::on_event( const MsgFault &msg )
  {
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedRamp::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
