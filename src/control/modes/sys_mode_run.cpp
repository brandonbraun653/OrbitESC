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
    //LOG_INFO_IF( !Chimera::System::inISR(), "Exiting Run state\r\n" );
  }

  etl::fsm_state_id_t Engaged::on_enter_state()
  {
    //LOG_INFO_IF( !Chimera::System::inISR(), "Entering Run state\r\n" );
    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Engaged::on_event( const MsgEmergencyHalt &msg )
  {
    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Engaged::on_event( const MsgDisengage &msg )
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
