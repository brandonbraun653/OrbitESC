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
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/control/modes/sys_mode_armed.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_led.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  Static Methods
  ---------------------------------------------------------------------------*/

  /**
   * @brief Validates the supply range.
   *
   * This function checks if the supply voltage is within the acceptable range.
   *
   * @return true if the supply voltage is within the acceptable range, false otherwise.
   */
  static bool armCheckVBusOk()
  {
    const float voltage = Orbit::Instrumentation::getSupplyVoltage();
    if( voltage < Orbit::Data::SysConfig.minArmVoltage )
    {
      LOG_WARN( "Cannot engage ARMED state. Power supply [%.2fV] < [%.2fV].", voltage, Orbit::Data::SysConfig.minArmVoltage );
      return false;
    }
    else if( voltage > Orbit::Data::SysConfig.maxArmVoltage )
    {
      LOG_WARN( "Cannot engage ARMED state. Power supply [%.2fV] > [%.2fV]", voltage, Orbit::Data::SysConfig.maxArmVoltage );
      return false;
    }

    return true;
  }


  /**
   * Checks if the system is currently in the expected subroutine.
   *
   * @return true if the system is in the expected subroutine, false otherwise.
   */
  static bool armCheckStartingRoutine()
  {
    using namespace Orbit::Control::Subroutine;

    if( const auto sub = getActiveSubroutine(); sub != Routine::IDLE)
    {
      LOG_WARN( "Cannot engage ARMED state. Subroutine [%s] is active.", getSubroutineName( sub ) );
      return false;
    }

    return true;
  }

  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/

  void Armed::on_exit_state()
  {
    LED::clearChannel( LED::Channel::ARMED );
    LOG_INFO( "Exiting ARMED state" );
  }

  etl::fsm_state_id_t Armed::on_enter_state()
  {
    using namespace Subroutine;

    /*-------------------------------------------------------------------------
    Perform entrance checks
    -------------------------------------------------------------------------*/
    if( !armCheckVBusOk() || !armCheckStartingRoutine() )
    {
      return ModeId::IDLE;
    }

    /*-------------------------------------------------------------------------
    Instruct the motor control subsystem to begin the rotor alignment detection
    procedure.
    -------------------------------------------------------------------------*/
    // TODO BMB: Somehow this is triggering a full system reset when the external
    // TODO BMB: power supply isn't connected. What's going on here?
    Control::Field::powerUp();
    Control::Speed::powerUp();

    // if( !switchRoutine( Routine::ALIGNMENT_DETECTION ) )
    // {
    //   LOG_ERROR( "Failed to start alignment detection routine" );
    //   return ModeId::IDLE;
    // }


    // if( !switchRoutine( Routine::ADC_SAMPLE_POINT_OPTIMIZER ) )
    // {
    //   LOG_ERROR( "Failed to start alignment detection routine" );
    //   return ModeId::IDLE;
    // }

    /*-------------------------------------------------------------------------
    Signal to the user that the system is armed
    -------------------------------------------------------------------------*/
    LED::setChannel( LED::Channel::ARMED );

    LOG_INFO( "Entered ARMED state" );
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t Armed::on_event( const MsgEngage &msg )
  {
    /*-------------------------------------------------------------------------
    Transition directly to the ENGAGED state. Let on_enter_state() do the work.
    -------------------------------------------------------------------------*/
    return ModeId::ENGAGED;
  }


  etl::fsm_state_id_t Armed::on_event( const MsgDisable &msg )
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

}    // namespace Orbit::Control::State
