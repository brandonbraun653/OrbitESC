/******************************************************************************
 *  File Name:
 *    sys_mode_fault.cpp
 *
 *  Description:
 *    Armed state
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_fault.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Fault::on_exit_state()
  {
    LOG_INFO( "Exiting Fault state\r\n" );
  }

  etl::fsm_state_id_t Fault::on_enter_state()
  {
    LOG_INFO( "Entering Fault state\r\n" );
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Fault::on_event( const MsgEmergencyHalt &msg )
  {
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Fault::on_event( const MsgArm &msg )
  {
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Fault::on_event( const MsgFault &msg )
  {
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Fault::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
