/******************************************************************************
 *  File Name:
 *    sys_mode_idle.cpp
 *
 *  Description:
 *    Idle state
 *
 *  2022-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/system>
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/control/modes/sys_mode_idle.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/monitor/orbit_monitors.hpp>
#include <src/simulator/sim_motor.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr bool DEBUG_MODULE = true;

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  void Idle::on_exit_state()
  {
    LOG_INFO( "Exiting IDLE state" );
  }


  etl::fsm_state_id_t Idle::on_enter_state()
  {
    LOG_INFO( "Entering IDLE state" );

    /*-------------------------------------------------------------------------
    Power down the motor control drivers
    -------------------------------------------------------------------------*/
    Control::Field::powerDn();
    Control::Speed::powerDn();

    if( !Subroutine::switchRoutine( Subroutine::Routine::IDLE ) )
    {
      LOG_ERROR( "Failed to start IDLE routine" );
    }

    /*-------------------------------------------------------------------------
    Disconnect the virtualized motor from the system
    -------------------------------------------------------------------------*/
    #if defined( SIMULATOR )
    Orbit::Sim::Motor::disconnect();
    #endif

    // /*-------------------------------------------------------------------------
    // Disable the drive signals going to the motor
    // -------------------------------------------------------------------------*/
    // Orbit::Control::FOC& driver = get_fsm_context();

    // driver.mTimerDriver.disableOutput();

    // /*-------------------------------------------------------------------------
    // Enable the system monitors
    // -------------------------------------------------------------------------*/
    // for( auto mon : Orbit::Monitor::MonitorArray )
    // {
    //   mon->setEngageState( Orbit::Monitor::EngageState::ACTIVE );
    // }

    // /*-------------------------------------------------------------------------
    // Clear any previous fault indicators
    // -------------------------------------------------------------------------*/
    // LED::clearChannel( LED::Channel::FAULT );

    return ModeId::IDLE;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgDisable & )
  {
    return ModeId::IDLE;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgArm &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the ARMED state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t Idle::on_event( const MsgFault &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the FAULT state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::FAULT;
  }


  etl::fsm_state_id_t Idle::on_event_unknown( const etl::imessage &msg )
  {
    get_fsm_context().logUnhandledMessage( msg );
    return this->No_State_Change;
  }

}  // namespace Orbit::Control::State
