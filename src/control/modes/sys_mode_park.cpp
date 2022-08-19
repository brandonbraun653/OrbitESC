/******************************************************************************
 *  File Name:
 *    sys_mode_park.cpp
 *
 *  Description:
 *    Armed state
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_park.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr bool DEBUG_MODULE = true;

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void EngagedPark::on_exit_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Exiting PARK state\r\n" );
  }

  etl::fsm_state_id_t EngagedPark::on_enter_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE, "Entered PARK state\r\n" );
    return ModeId::ENGAGED_PARK;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgRamp &msg )
  {
    /*-------------------------------------------------------------------------
    Prepare the ramp controller
    -------------------------------------------------------------------------*/
    LOG_ERROR( "Fill in the ramp state change\r\n" );

    /*-------------------------------------------------------------------------
    Stop the park controller
    -------------------------------------------------------------------------*/
    return ModeId::ENGAGED_RAMP;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgEmergencyHalt &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgFault &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgDisengage &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the ARMED state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t EngagedPark::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
