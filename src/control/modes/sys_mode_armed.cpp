/******************************************************************************
 *  File Name:
 *    sys_mode_armed.cpp
 *
 *  Description:
 *    State machine control logic for the ARMED state
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/system>
#include <src/control/modes/sys_mode_armed.hpp>
#include <src/control/current_control.hpp>
#include <src/control/speed_control.hpp>
#include <src/core/hw/orbit_led.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Armed::on_exit_state()
  {
    LOG_INFO( "Exiting ARMED state" );
    LED::clearChannel( LED::Channel::ARMED );
  }

  etl::fsm_state_id_t Armed::on_enter_state()
  {
    /*-------------------------------------------------------------------------
    Power up the motor control drivers
    -------------------------------------------------------------------------*/
    Control::Field::powerUp();
    Control::Speed::powerUp();

    /*-------------------------------------------------------------------------
    Signal to the user that the system is armed
    -------------------------------------------------------------------------*/
    LED::setChannel( LED::Channel::ARMED );

    LOG_INFO( "Entered ARMED state" );
    return ModeId::ARMED;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgEmergencyHalt &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgDisarm &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the IDLE state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::IDLE;
  }

  etl::fsm_state_id_t Armed::on_event( const MsgFault &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }

  etl::fsm_state_id_t Armed::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
