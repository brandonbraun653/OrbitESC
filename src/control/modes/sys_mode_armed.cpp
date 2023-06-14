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
#include <Chimera/system>
#include <src/control/modes/sys_mode_armed.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr bool DEBUG_MODULE = true;

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Armed::on_exit_state()
  {
    LOG_TRACE_IF( DEBUG_MODULE && !Chimera::System::inISR(), "Exiting ARMED state\r\n" );
  }

  etl::fsm_state_id_t Armed::on_enter_state()
  {
    // Orbit::Control::FOC& driver = get_fsm_context();

    // driver.mTimerDriver.disableOutput();
    // driver.mTimerDriver.setForwardCommState( 0 );
    // driver.mTimerDriver.setPhaseDutyCycle( 0.0f, 0.0f, 0.0f );
    // driver.mTimerDriver.enableOutput();

    // LOG_TRACE_IF( DEBUG_MODULE && !Chimera::System::inISR(), "Entered ARMED state\r\n" );
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
