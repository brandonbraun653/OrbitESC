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
  State Class
  ---------------------------------------------------------------------------*/
  etl::fsm_state_id_t EngagedPark::on_enter_state()
  {
    LOG_INFO( "Entering Park state\r\n" );
    return ModeId::ENGAGED_PARK;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgEmergencyHalt &msg )
  {
    return ModeId::ENGAGED_PARK;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgDisengage &msg )
  {
    return ModeId::ENGAGED_PARK;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgRamp &msg )
  {
    return ModeId::ENGAGED_PARK;
  }


  etl::fsm_state_id_t EngagedPark::on_event( const MsgFault &msg )
  {
    return ModeId::ENGAGED_PARK;
  }


  etl::fsm_state_id_t EngagedPark::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
