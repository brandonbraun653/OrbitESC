/******************************************************************************
 *  File Name:
 *    sys_mode_run.cpp
 *
 *  Description:
 *    Armed state
 *
 *  2022-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/system>
#include <src/control/modes/sys_mode_run.hpp>
#include <src/control/subroutines/interface.hpp>

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

    if( !Subroutine::switchRoutine( Subroutine::Routine::OPEN_LOOP_RAMP_FOC ) )
    {
      LOG_ERROR( "Failed to start motor ramp routine" );
      return this->No_State_Change;
    }

    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Engaged::on_event( const MsgDisable &msg )
  {
    LOG_INFO( "RUN state received Disable event. Transitioning to ARMED mode." );

    if( !Subroutine::switchRoutine( Subroutine::Routine::IDLE ) )
    {
      LOG_ERROR( "Failed to start motor idle routine" );
      return this->No_State_Change;
    }

    return ModeId::IDLE;
  }


  etl::fsm_state_id_t Engaged::on_event( const MsgFault &msg )
  {
    LOG_INFO( "RUN state received Fault event. Transitioning to FAULT mode." );
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Engaged::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
