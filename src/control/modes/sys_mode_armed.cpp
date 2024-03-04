/******************************************************************************
 *  File Name:
 *    sys_mode_armed.cpp
 *
 *  Description:
 *    State machine control logic for the ARMED state
 *
 *  2022-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/system>
#include <src/control/foc_math.hpp>
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/control/modes/sys_mode_armed.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/simulator/sim_motor.hpp>

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
    Connect the virtualized motor to the system
    -------------------------------------------------------------------------*/
    #if defined( SIMULATOR )
    Orbit::Sim::Motor::Parameters params;
    CLEAR_STRUCT( params );

    // Parameters taken from https://ieeexplore.ieee.org/document/6687627 (Table 1)
    params.r          = 0.982f;       // Stator resistance
    params.ld         = 2.9e-3f;      // D-axis inductance
    params.lq         = 3.0e-3f;      // Q-axis inductance
    params.lpm        = 0.075;        // Permanent magnet flux linkage
    params.pole_pairs = 4;            // Number of pole pairs
    params.J          = 0.425e-3f;    // Rotational inertia
    params.v_max_adc  = 3.3f;         // Max voltage that ADC can measure
    params.km         = 1.5f * params.pole_pairs;

    Orbit::Sim::Motor::connect( params );
    #endif

    /*-------------------------------------------------------------------------
    Instruct the motor control subsystem to begin the rotor alignment detection
    procedure.
    -------------------------------------------------------------------------*/
    // TODO BMB: Somehow this is triggering a full system reset when the external
    // TODO BMB: power supply isn't connected. What's going on here? Do I need more
    // TODO BMB: bulk capacitance?
    Control::Field::powerUp();
    Control::Speed::powerUp();

    // if( !switchRoutine( Routine::ALIGNMENT_DETECTION ) )
    // {
    //   LOG_ERROR( "Failed to start alignment detection routine" );
    //   return ModeId::IDLE;
    // }

    // Temporarily drive a fixed park vector
    Data::SysControl.parkTheta = DEG_TO_RAD( 1.0f * 60.0f );

    /*-------------------------------------------------------------------------
    Signal to the user that the system is armed
    -------------------------------------------------------------------------*/
    LED::setChannel( LED::Channel::ARMED );

    LOG_INFO( "Entered ARMED state" );
    return ModeId::ARMED;
  }


  etl::fsm_state_id_t Armed::on_event( const MsgEngage &msg )
  {
    using namespace Subroutine;

    /*-------------------------------------------------------------------------
    Validate we can transition. It's expected the alignment detection routine
    will have completed before we can engage.
    -------------------------------------------------------------------------*/
    if( auto active_routine = getActiveSubroutine(); active_routine != Routine::IDLE )
    {
      LOG_WARN( "ARM -> ENGAGE fail. Subroutine [%s] is active.", getSubroutineName( active_routine ) );
      return this->No_State_Change;
    }

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
