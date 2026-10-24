/******************************************************************************
 *  File Name:
 *    foc_driver.cpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <cmath>
#include <src/config/orbit_esc_cfg.hpp>
#include <src/config/bsp/board_map.hpp>
#include <src/control/foc_driver.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/modes/sys_mode_armed.hpp>
#include <src/control/modes/sys_mode_base.hpp>
#include <src/control/modes/sys_mode_fault.hpp>
#include <src/control/modes/sys_mode_idle.hpp>
#include <src/control/modes/sys_mode_run.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/utility.hpp>
#include <src/monitor/orbit_monitors.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/interface/inc/timer>
#endif /* EMBEDDED */

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* SEGGER_SYS_VIEW */


namespace Orbit::Control::FOC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static StateMachine                                      s_ctrl_fsm;
  static SuperState                                        mState;         /**< Entire FOC subsystem state */
  static std::array<etl::ifsm_state *, ModeId::NUM_STATES> mFSMStateArray; /**< Storage for the FSM state controllers */


  /*---------------------------------------------------------------------------
  StateMachine Implementation
  ---------------------------------------------------------------------------*/
  StateMachine::StateMachine() : fsm( MOTOR_STATE_ROUTER_ID )
  {
  }

  void StateMachine::logUnhandledMessage( const etl::imessage &msg )
  {
    LOG_WARN( "%s message not handled from state %s", getMessageString( msg.get_message_id() ).data(),
              getModeString( get_state_id() ).data() );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    /*-------------------------------------------------------------------------
    Validate the configuration
    -------------------------------------------------------------------------*/
    // for ( auto &txfrFunc : cfg.txfrFuncs )
    // {
    //   ( void )txfrFunc;
    //   RT_DBG_ASSERT( txfrFunc != nullptr );
    // }

    // /*-------------------------------------------------------------------------
    // Initialize the FOC state
    // -------------------------------------------------------------------------*/
    // mState.clear();
    // mConfig.clear();

    // mConfig            = cfg;
    // mState.motorParams = motorParams;


    // /*-------------------------------------------------------------------------
    // Initialize eigenvalues for the current controller
    // -------------------------------------------------------------------------*/
    // mState.emfObserver.d = ( -motorParams.Rs / motorParams.Ls ) * 5.0f;

    // ControllerData* ctl = &mState.motorCtl;

    // ctl->clear();

    // ctl->Dpid.SetPoint    = 0.0f;
    // ctl->Dpid.OutMinLimit = 0.0f;
    // ctl->Dpid.OutMaxLimit = 1.0f;
    // ctl->DFIR = Math::FIR<float, 15>( { -0.00124568841F, 0.00147019443F, 0.0123328818F, 0.00110139197F, -0.0499843247F,
    //                                     -0.0350326933F, 0.164325342F, 0.407320708F, 0.407320708F, 0.164325342F,
    //                                     -0.0350326933F, -0.0499843247F, 0.00110139197F, 0.0123328818F, 0.00147019443F,
    //                                     -0.00124568841F } );
    // ctl->DFIR.initialize();

    // ctl->Qpid.SetPoint    = 0.0f;
    // ctl->Qpid.OutMinLimit = 0.0f;
    // ctl->Qpid.OutMaxLimit = 1.0f;
    // ctl->QFIR = Math::FIR<float, 15>( { -0.00124568841F, 0.00147019443F, 0.0123328818F, 0.00110139197F, -0.0499843247F,
    //                                     -0.0350326933F, 0.164325342F, 0.407320708F, 0.407320708F, 0.164325342F,
    //                                     -0.0350326933F, -0.0499843247F, 0.00110139197F, 0.0123328818F, 0.00147019443F,
    //                                     -0.00124568841F } );
    // ctl->QFIR.initialize();
    // ctl->Qpid.setTunings( 5.0f, 2.0f, 0.3f, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );

    /*-------------------------------------------------------------------------
    Initialize the finite state machine
    -------------------------------------------------------------------------*/
    mFSMStateArray.fill( nullptr );
    mFSMStateArray[ ModeId::IDLE ]    = new State::Idle();
    mFSMStateArray[ ModeId::ARMED ]   = new State::Armed();
    mFSMStateArray[ ModeId::FAULT ]   = new State::Fault();
    mFSMStateArray[ ModeId::ENGAGED ] = new State::Engaged();

    /* Initialize the FSM. First state will be ModeId::IDLE. */
    s_ctrl_fsm.set_states( mFSMStateArray.data(), mFSMStateArray.size() );
    s_ctrl_fsm.start();

    // mInitialized = true;
  }


  void process()
  {
  }


  int sendSystemEvent( const EventId_t event )
  {
    /*-------------------------------------------------------------------------
    Send the correct message into the state machine
    -------------------------------------------------------------------------*/
    switch( event )
    {
      case EventId::ARM:
        s_ctrl_fsm.receive( MsgArm() );
        break;

      case EventId::ENGAGE:
        s_ctrl_fsm.receive( MsgEngage() );
        break;

      case EventId::DISABLE:
        s_ctrl_fsm.receive( MsgDisable() );
        break;

      case EventId::FAULT:
        s_ctrl_fsm.receive( MsgFault() );
        break;

      default:
        return -1;
        break;
    }

    return 0;
  }


  int setSpeedRef( const float ref )
  {
    return 0;
  }


  const SuperState &dbgGetState()
  {
    return mState;
  }


  ModeId_t currentMode()
  {
    return s_ctrl_fsm.get_state_id();
  }


  void driveTestSignal( const uint8_t commCycle, const float dutyCycle )
  {
    /*-------------------------------------------------------------------------
    Ensure we're in the ARMED state before attempting anything. This guarantees
    our protections are running but the output is disabled.
    -------------------------------------------------------------------------*/
    if( currentMode() != ModeId::ARMED )
    {
      return;
    }
  }


  /**
   * @brief Calculates back-EMF estimates along the D and Q axes
   * @see https://ieeexplore.ieee.org/document/530286
   *
   * @param dt  The time in seconds since the last call to this function
   */
  void stepEMFObserver( const float dt )
  {
    // /*-------------------------------------------------------------------------
    // Alias equation variables to make everything easier to read
    // -------------------------------------------------------------------------*/
    // const float d  = mState.emfObserver.d;
    // const float Ls = mState.motorParams.Ls;
    // const float Rs = mState.motorParams.Rs;
    // const float Wr = mState.motorCtl.velEstRad;
    // const float Iq = mState.motorCtl.Iq;
    // const float Id = mState.motorCtl.Id;
    // const float Vq = mState.motorCtl.Vq;
    // const float Vd = mState.motorCtl.Vd;

    // /*-------------------------------------------------------------------------
    // Update the D-Q axis observer state space model (Eq. 11)
    // -------------------------------------------------------------------------*/
    // const float q_tmp = ( ( Rs + ( d * Ls ) ) * Iq ) - Vq;
    // const float d_tmp = ( ( Rs + ( d * Ls ) ) * Id ) - Vd;

    // mState.emfObserver.z1_dot = ( d * dt * mState.emfObserver.z1 ) + dt * ( ( d * q_tmp ) - ( Wr * d_tmp ) );
    // mState.emfObserver.z2_dot = ( d * dt * mState.emfObserver.z2 ) + dt * ( ( Wr * q_tmp ) + ( d * d_tmp ) );

    // /*-------------------------------------------------------------------------
    // Calculate the estimated EMF (Eq. 12)
    // -------------------------------------------------------------------------*/
    // mState.emfObserver.Eq_est = mState.emfObserver.z1_dot + ( d * Ls * Iq ) - ( Ls * Wr * Id );
    // mState.emfObserver.Ed_est = mState.emfObserver.z2_dot + ( Ls * Wr * Iq ) + ( d * Ls * Id );

    // /*-------------------------------------------------------------------------
    // Update the EMF observer state. Note that Z1 dot already has the sample
    // time baked in from Equation 11.
    // -------------------------------------------------------------------------*/
    // mState.emfObserver.z1 = mState.emfObserver.z1_dot;
    // mState.emfObserver.z2 = mState.emfObserver.z2_dot;
  }

}    // namespace Orbit::Control::FOC
