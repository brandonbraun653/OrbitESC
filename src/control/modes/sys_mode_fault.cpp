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
#include <Chimera/system>
#include <src/control/modes/sys_mode_fault.hpp>
#include <src/core/hw/orbit_led.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Fault::on_exit_state()
  {
    LOG_INFO_IF( !Chimera::System::inISR(), "Exiting Fault state\r\n" );
  }

  etl::fsm_state_id_t Fault::on_enter_state()
  {
    // /*-------------------------------------------------------------------------
    // Immediately stop the motor drive signals
    // -------------------------------------------------------------------------*/
    // get_fsm_context().mTimerDriver.emergencyBreak();
    // get_fsm_context().mState.motorCtl.isrCtlActive = false;

    // /*-------------------------------------------------------------------------
    // Signal to the user that something is wrong
    // -------------------------------------------------------------------------*/
    // LED::setChannel( LED::Channel::FAULT );

    // /*-------------------------------------------------------------------------
    // // TODO: Signal to controller over CAN that something has happened
    // -------------------------------------------------------------------------*/

    // /*-------------------------------------------------------------------------
    // Do non-critical tasks
    // -------------------------------------------------------------------------*/
    // LOG_INFO_IF( !Chimera::System::inISR(), "Entering Fault state\r\n" );
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
