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
#include <Chimera/system>
#include <src/control/modes/sys_mode_run.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Engaged::on_exit_state()
  {
    LOG_INFO( "Exiting RUN state" );
  }

  etl::fsm_state_id_t Engaged::on_enter_state()
  {
    LOG_INFO( "Entering RUN state" );
    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Engaged::on_event( const MsgDisable &msg )
  {
    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Engaged::on_event( const MsgFault &msg )
  {
    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Engaged::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
