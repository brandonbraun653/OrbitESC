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
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/tasks/tsk_ctrl_sys.hpp>
#include <src/core/utility.hpp>

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
    RampControl *const pCtrl = &get_fsm_context().mState.motorController.ramp;
    pCtrl->clear();

    /* Set up static data */
    pCtrl->comState            = Orbit::Data::DFLT_STATOR_ALIGN_COMM_PHASE;
    pCtrl->phaseDutyCycle[ 0 ] = Orbit::Data::DFLT_RAMP_DRIVE_STRENGTH_PCT;
    pCtrl->phaseDutyCycle[ 1 ] = Orbit::Data::DFLT_RAMP_DRIVE_STRENGTH_PCT;
    pCtrl->phaseDutyCycle[ 2 ] = Orbit::Data::DFLT_RAMP_DRIVE_STRENGTH_PCT;

    pCtrl->rampRate  = 2;    // RPM per Orbit::Tasks::CTRLSYS::PERIOD_MS
    pCtrl->finalRPM  = 1000;
    pCtrl->targetRPM = 5;
    pCtrl->minDwellCycles =
        Utility::comCycleCount( Data::DFLT_STATOR_PWM_FREQ_HZ, Data::DFLT_ROTOR_NUM_POLES, pCtrl->finalRPM );

    /* Set up dynamic data */
    pCtrl->cycleCount = 0;
    pCtrl->cycleRef   = Utility::comCycleCount( Data::DFLT_STATOR_PWM_FREQ_HZ, Data::DFLT_ROTOR_NUM_POLES, pCtrl->targetRPM );

    /*-------------------------------------------------------------------------
    Start the park controller
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
