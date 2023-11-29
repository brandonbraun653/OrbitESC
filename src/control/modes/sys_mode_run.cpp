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
#include <src/core/hw/orbit_instrumentation.hpp>

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

    // TODO BMB: Configure this threshold with a parameter
    if( Orbit::Instrumentation::getSupplyVoltage() < 10.0f )
    {
      LOG_WARN( "Cannot engage run state. Supply voltage is too low." );
      return this->No_State_Change;
    }


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
