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
  State Class
  ---------------------------------------------------------------------------*/
  void Idle::on_exit_state()
  {
    LOG_INFO( "Exiting Idle state\r\n" );
  }


  etl::fsm_state_id_t Idle::on_enter_state()
  {
    LOG_INFO( "Entering Idle state\r\n" );
    return ModeId::IDLE;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgEmergencyHalt &msg )
  {
    return ModeId::IDLE;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgArm &msg )
  {
    LOG_INFO( "Got Arm event\r\n" );
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgFault &msg )
  {
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Idle::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
