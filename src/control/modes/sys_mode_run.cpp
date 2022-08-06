/******************************************************************************
 *  File Name:
 *    sys_mode_run.cpp
 *
 *  Description:
 *    Armed state
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_run.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  etl::fsm_state_id_t EngagedRun::on_enter_state()
  {
    return StateId::ENGAGED_RUN;
  }


  etl::fsm_state_id_t EngagedRun::on_event( const MsgEmergencyHalt &msg )
  {
    return StateId::ENGAGED_RUN;
  }


  etl::fsm_state_id_t EngagedRun::on_event( const MsgDisengage &msg )
  {
    return StateId::ENGAGED_RUN;
  }


  etl::fsm_state_id_t EngagedRun::on_event( const MsgFault &msg )
  {
    return StateId::ENGAGED_RUN;
  }


  etl::fsm_state_id_t EngagedRun::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
