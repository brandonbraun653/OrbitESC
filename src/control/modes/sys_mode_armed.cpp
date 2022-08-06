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
  State Class
  ---------------------------------------------------------------------------*/
  etl::fsm_state_id_t Armed::on_enter_state()
  {
    return StateId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgEmergencyHalt &msg )
  {
    return StateId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgDisarm &msg )
  {
    return StateId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgAlign &msg )
  {
    return StateId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgFault &msg )
  {
    return StateId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
