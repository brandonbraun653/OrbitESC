/******************************************************************************
 *  File Name:
 *    foc_driver.cpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <cmath>
#include <src/config/bsp/board_map.hpp>
#include <src/control/foc_driver.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/modes/sys_mode_armed.hpp>
#include <src/control/modes/sys_mode_base.hpp>
#include <src/control/modes/sys_mode_fault.hpp>
#include <src/control/modes/sys_mode_idle.hpp>
#include <src/control/modes/sys_mode_park.hpp>
#include <src/control/modes/sys_mode_ramp.hpp>
#include <src/control/modes/sys_mode_run.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/utility.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/interface/inc/timer>

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif  /* SEGGER_SYS_VIEW */
#endif  /* EMBEDDED */



namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  FOC FOCDriver;

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  FOC::FOC() : fsm( MOTOR_STATE_ROUTER_ID )
  {
  }

  FOC::~FOC()
  {
    mTimerDriver.emergencyBreak();
  }


  int FOC::initialize( const FOCConfig &cfg, const MotorParameters &motorParams )
  {
    /*-------------------------------------------------------------------------
    Validate the configuration
    -------------------------------------------------------------------------*/
    for ( auto &txfrFunc : cfg.txfrFuncs )
    {
      ( void )txfrFunc;
      RT_DBG_ASSERT( txfrFunc != nullptr );
    }

    /*-------------------------------------------------------------------------
    Initialize the FOC state
    -------------------------------------------------------------------------*/
    mState.clear();
    mConfig.clear();

    mConfig            = cfg;
    mState.motorParams = motorParams;

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this class's ISR handler
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<FOC, &FOC::adcISRTxfrComplete>( *this );

    mADCDriver = Chimera::ADC::getDriver( cfg.adcSource );
    mADCDriver->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    /*-------------------------------------------------------------------------
    Get the DC offset of current shunt resistors when the motor is at rest
    -------------------------------------------------------------------------*/
    Chimera::ADC::Sample sample;

    sample                                                    = mADCDriver->sampleChannel( Orbit::IO::Analog::adcPhaseA );
    mState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].dcOffset = mADCDriver->toVoltage( sample );

    sample                                                    = mADCDriver->sampleChannel( Orbit::IO::Analog::adcPhaseB );
    mState.adcBuffer[ ADC_CH_MOTOR_PHASE_B_CURRENT ].dcOffset = mADCDriver->toVoltage( sample );

    /*-------------------------------------------------------------------------
    Initialize eigenvalues for the current controller
    -------------------------------------------------------------------------*/
    mState.emfObserver.d = ( -motorParams.Rs / motorParams.Ls ) * 5.0f;

    /*-------------------------------------------------------------------------
    Configure the Advanced Timer for center-aligned 3-phase PWM
    -------------------------------------------------------------------------*/
    Chimera::Timer::Inverter::DriverConfig pwm_cfg;

    pwm_cfg.clear();
    pwm_cfg.coreCfg.instance    = Chimera::Timer::Instance::TIMER1;
    pwm_cfg.coreCfg.clockSource = Chimera::Clock::Bus::SYSCLK;
    pwm_cfg.coreCfg.baseFreq    = 40'000'000.0f;
    pwm_cfg.coreCfg.tolerance   = 1.0f;
    pwm_cfg.adcPeripheral       = cfg.adcSource;
    pwm_cfg.adcTriggerOffsetNs  = 50.0f;
    pwm_cfg.adcTriggerSignal    = Chimera::Timer::Trigger::Signal::TRIG_SIG_5;
    pwm_cfg.breakIOLevel        = Chimera::GPIO::State::LOW;
    pwm_cfg.deadTimeNs          = 250.0f;
    pwm_cfg.pwmFrequency =
        Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ * 2.0f;    // TODO BMB: For some reason HW output is divided by 2

    RT_HARD_ASSERT( Chimera::Status::OK == mTimerDriver.init( pwm_cfg ) );

    mTimerDriver.setPhaseDutyCycle( 10.0f, 10.0f, 10.0f );
    mTimerDriver.enableOutput();

    /*-------------------------------------------------------------------------
    Configure the Speed control outer loop update timer
    -------------------------------------------------------------------------*/
    Chimera::Function::Opaque isrFunc = Chimera::Function::Opaque::create<FOC, &FOC::timer_isr_speed_controller>( *this );

    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = 10.0f;
    trig_cfg.isrCallback            = isrFunc;
    trig_cfg.coreConfig.instance    = Chimera::Timer::Instance::TIMER2;
    trig_cfg.coreConfig.baseFreq    = 100'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == mSpeedCtrlTrigger.init( trig_cfg ) );
    mSpeedCtrlTrigger.enable();

    /*-------------------------------------------------------------------------
    Initialize the finite state machine
    -------------------------------------------------------------------------*/
    mFSMStateArray.fill( nullptr );
    mFSMStateArray[ ModeId::IDLE ]         = new State::Idle();
    mFSMStateArray[ ModeId::ARMED ]        = new State::Armed();
    mFSMStateArray[ ModeId::FAULT ]        = new State::Fault();
    mFSMStateArray[ ModeId::ENGAGED_PARK ] = new State::EngagedPark();
    mFSMStateArray[ ModeId::ENGAGED_RAMP ] = new State::EngagedRamp();
    mFSMStateArray[ ModeId::ENGAGED_RUN ]  = new State::EngagedRun();

    /* Initialize the FSM. First state will be ModeId::IDLE. */
    this->set_states( mFSMStateArray.data(), mFSMStateArray.size() );
    this->start();

    /*-------------------------------------------------------------------------
    Initialize the periodic behavior lookup table
    https://stackoverflow.com/a/1486279/8341975
    -------------------------------------------------------------------------*/
    mRunFuncArray.fill( nullptr );
    mRunFuncArray[ ModeId::ARMED ]        = &FOC::onArmed;
    mRunFuncArray[ ModeId::FAULT ]        = &FOC::onFault;
    mRunFuncArray[ ModeId::ENGAGED_PARK ] = &FOC::onPark;
    mRunFuncArray[ ModeId::ENGAGED_RAMP ] = &FOC::onRamp;
    mRunFuncArray[ ModeId::ENGAGED_RUN ]  = &FOC::onRun;

    return 0;
  }


  void FOC::run()
  {
    /*-------------------------------------------------------------------------
    Nothing crazy, just map into the appropriate function for the current mode.
    Should be a little faster/smaller than a switch case.
    -------------------------------------------------------------------------*/
    const auto mode = this->currentMode();
    if ( mRunFuncArray[ mode ] )
    {
      ( this->*mRunFuncArray[ mode ] )();
    }
  }


  int FOC::arm()
  {
    this->receive( MsgArm() );
    return ( this->currentMode() == ModeId::ARMED ) ? 0 : -1;
  }


  int FOC::disarm()
  {
    this->receive( MsgDisarm() );
    return ( this->currentMode() == ModeId::IDLE ) ? 0 : -1;
  }


  int FOC::engage()
  {
    this->receive( MsgAlign() );

    auto mode = this->currentMode();
    return ( ( mode == ModeId::ENGAGED_PARK ) || ( mode == ModeId::ENGAGED_RAMP ) || ( mode == ModeId::ENGAGED_RUN ) ) ? 0 : -1;
  }


  int FOC::disengage()
  {
    this->receive( MsgDisengage() );
    return ( this->currentMode() == ModeId::ARMED ) ? 0 : -1;
  }


  int FOC::setSpeedRef( const float ref )
  {
    //mState.motorCtl.run.speedRefRad = ref;
    return 0;
  }


  int FOC::emergencyStop()
  {
    this->receive( MsgEmergencyHalt() );
    return ( this->currentMode() == ModeId::IDLE ) ? 0 : -1;
  }


  void FOC::lastSensorData( ADCSensorBuffer &data )
  {
    data = mState.adcBuffer;
  }


  const SuperState &FOC::dbgGetState() const
  {
    return mState;
  }


  ModeId_t FOC::currentMode() const
  {
    return this->get_state_id();
  }


  void FOC::logUnhandledMessage( const etl::imessage &msg )
  {
    LOG_WARN( "%s message not handled from state %s\r\n", getMessageString( msg.get_message_id() ).data(),
              getModeString( get_state_id() ).data() );
  }


  void FOC::adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr )
  {
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    /*-------------------------------------------------------------------------
    Convert the ADC data to measured values
    -------------------------------------------------------------------------*/
    static constexpr float COUNTS_TO_VOLTS = 3.3f / 4096.0f;    // Vref = 3.3V, 12-bit ADC
    const uint32_t timestamp_us = Chimera::micros();
    const float f_timestamp_us = static_cast<float>( timestamp_us );

    for ( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      mState.adcBuffer[ i ].measured     = static_cast<float>( isr.samples[ i ] ) * COUNTS_TO_VOLTS;
      mState.adcBuffer[ i ].converted    = mConfig.txfrFuncs[ i ]( mState.adcBuffer[ i ].measured );
      mState.adcBuffer[ i ].sampleTimeUs = timestamp_us;
    }

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    if( !mState.motorCtl.isrCtlActive )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Update the cycle-by-cycle current control loop
    -------------------------------------------------------------------------*/
    const float cycle_dt = f_timestamp_us - mState.motorCtl.last_current_update_us;
    stepIControl( cycle_dt );
    mState.motorCtl.last_current_update_us = timestamp_us;

    /*-------------------------------------------------------------------------
    Update the Position/Speed estimators
    -------------------------------------------------------------------------*/
    const float estimate_dt = f_timestamp_us - mState.motorCtl.last_estimate_update_us;

    if( estimate_dt >= mState.motorCtl.next_estimate_update_us )
    {
      mState.motorCtl.last_estimate_update_us = f_timestamp_us;
      mState.motorCtl.next_estimate_update_us = 5.0f * f_timestamp_us;
      stepEstimator( estimate_dt );
    }

    /**
     * Current Control Loop: (Ts)
     *  - Start with the physical system measurements
     *  - Using the last known speed and position estimates, calculate the dq_actual frame data
     *  - Generate the dq_error signals and push through their PID controllers
     *  - Using last known position estimates, do inverse park/clarke transform, generating PWM values for each phase
     *  - Using last known commutation state, apply PWM values to motor control output
     *
     * Position/Speed/Commutation Update Loop: (5*Ts)
     *  - Step the EMF observer to estimate the back EMF voltages in the dq frame
     *  - Update the speed/position estimates
     *    - If open loop control -> Use time integral of current set-point (Iqr)
     *    - If closed loop ctrl  -> Use the back EMF estimates
     *      - Pos:   Eq. 18
     *      - Speed: Eq. 22
     *  - Update the commutation state based on the current position and speed. (Is this best?)
     */

    /*-------------------------------------------------------------------------
    Estimate the rotor position
    -------------------------------------------------------------------------*/

      /*-----------------------------------------------------------------------
      Move the sampled phase currents through the Clarke-Park transform
      -----------------------------------------------------------------------*/
      // const auto clarke = Math::clarke_transform( mState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].converted,
      //                                             mState.adcBuffer[ ADC_CH_MOTOR_PHASE_B_CURRENT ].converted );

      // const auto park = Math::park_transform( clarke, mState.motorController.posEstRad );

      // /*-----------------------------------------------------------------------
      // Do the estimation
      // -----------------------------------------------------------------------*/
      // /* Update the motor state data from previous calculations */
      // mState.motorController.Iq  = park.q;
      // mState.motorController.Id  = park.d;
      // mState.motorController.Vdd = mState.adcBuffer[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].converted;

      // /* Step the EMF observer */
      // const float dt = US_TO_SEC( timestamp_us - mState.emfObserver.last_update_us );
      // stepEMFObserver( dt );
      // mState.emfObserver.last_update_us = timestamp_us;

      // /* Calculate the estimated rotor position (Eq. 18) */
      // mState.motorController.posEstRad = Math::fast_atan2_with_norm( -mState.emfObserver.Ed_est, mState.emfObserver.Eq_est );
      // if ( mState.motorController.velEstRad < 0.0f )
      // {
      //   mState.motorController.posEstRad += Math::M_PI_F;
      // }

    /*-------------------------------------------------------------------------
    Based on the current state, decide how to drive the output stage
    -------------------------------------------------------------------------*/
    // if ( current_mode == ModeId::ENGAGED_RUN ) {}
    // else if ( current_mode == ModeId::ENGAGED_RAMP )
    // {
    //   mTimerDriver.setForwardCommState( mState.motorController.ramp.comState );
    //   mTimerDriver.setPhaseDutyCycle( mState.motorController.ramp.phaseDutyCycle[ 0 ],
    //                                   mState.motorController.ramp.phaseDutyCycle[ 1 ],
    //                                   mState.motorController.ramp.phaseDutyCycle[ 2 ] );
    //   isr_rotor_ramp_controller();
    // }
    // else if ( current_mode == ModeId::ENGAGED_PARK )
    // {
    //   if ( mState.motorController.park.outputEnabled )
    //   {
    //     mTimerDriver.setPhaseDutyCycle( mState.motorController.park.phaseDutyCycle[ 0 ],
    //                                     mState.motorController.park.phaseDutyCycle[ 1 ],
    //                                     mState.motorController.park.phaseDutyCycle[ 2 ] );

    //     mTimerDriver.setForwardCommState( mState.motorController.park.activeComState );
    //   }
    //   else
    //   {
    //     mTimerDriver.setForwardCommState( 0 );
    //   }
    // }
    // Else nothing useful to do

    /*-------------------------------------------------------------------------
    Update the commanded controller output states
    -------------------------------------------------------------------------*/
    // const Math::ParkSpace park_space{ .d = mState.motorController.Vd, .q = mState.motorController.Vq };

    // auto  invPark = Math::inverse_park_transform( park_space, mState.motorController.posEstRad );
    // float a, b, c;

    // Math::inverse_clarke_transform( invPark, &a, &b, &c );

    // mTimerDriver.setPhaseDutyCycle( 10.0f, 10.0f, 10.0f );

    /*-------------------------------------------------------------------------
    Perform commutation
    -------------------------------------------------------------------------*/
    // Use position estimate to determine the next commutation state. I'm going
    // to guess dividing 2PI into six sectors where each sector represents
    // 60 degrees of rotation. The main question is how to determine which phase
    // correctly corresponds to the current position estimate?

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


  void FOC::timer_isr_speed_controller()
  {
    mSpeedCtrlTrigger.ackISR();

    /*-------------------------------------------------------------------------
    Don't run the speed controller loop if not in closed loop control
    -------------------------------------------------------------------------*/
    // if ( this->get_state_id() != EnumValue( ModeId::ENGAGED_RUN ) )
    // {
    //   return;
    // }

    // /*-------------------------------------------------------------------------
    // Update estimate of the rotor speed
    // -------------------------------------------------------------------------*/
    // const size_t timestamp = Chimera::micros();
    // const float  dTheta    = mState.motorCtl.posEstRad - mState.speedEstimator.pos_est_prev;
    // const float  wdt       = US_TO_SEC( timestamp - mState.speedEstimator.last_update_us );

    // mState.motorCtl.velEstRad = dTheta / wdt;

    /*-----------------------------------------------------------------------
    Update tracking variables
    -----------------------------------------------------------------------*/
    // mState.speedEstimator.pos_est_prev   = mState.motorCtl.posEstRad;
    // mState.speedEstimator.last_update_us = timestamp;

    // Reset the timer interrupt flag...
  }


  void FOC::isr_rotor_ramp_controller()
  {
    // RampControl *const pCtrl = &mState.motorCtl.ramp;

    // pCtrl->cycleCount++;
    // if ( pCtrl->cycleCount >= pCtrl->cycleRef )
    // {
    //   /*-----------------------------------------------------------------------
    //   Update the cycle references
    //   -----------------------------------------------------------------------*/
    //   pCtrl->cycleCount = 0;
    //   pCtrl->cycleRef   = Utility::comCycleCount( Data::DFLT_STATOR_PWM_FREQ_HZ, Data::DFLT_ROTOR_NUM_POLES, pCtrl->targetRPM );

    //   /*-----------------------------------------------------------------------
    //   Update the commutation state
    //   -----------------------------------------------------------------------*/
    //   pCtrl->comState++;
    //   if ( pCtrl->comState >= 7 )
    //   {
    //     pCtrl->comState = 1;
    //   }
    // }
  }


  /**
   * @brief Calculates back-EMF estimates along the D and Q axes
   * @see https://ieeexplore.ieee.org/document/530286
   *
   * @param dt  The time in seconds since the last call to this function
   */
  void FOC::stepEMFObserver( const float dt )
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


  void FOC::stepIControl( const float dt )
  {

  }


  void FOC::stepEstimator( const float dt )
  {

  }


  void FOC::onFault()
  {
  }

  /**
   * @brief Handles high level control of the ARMED state
   */
  void FOC::onArmed()
  {
    /*-------------------------------------------------------------------------
    For now, don't do anything. In the future, this would be a great place for
    motor parameter estimation and ADC tuning.
    -------------------------------------------------------------------------*/
  }


  /**
   * @brief Handles high level control of the ENGAGED_PARK state
   *
   * By the time this method has entered, the state machine controller will have
   * initialized the Park control data. All this method has to do is control the
   * temporal sequence for aligning the rotor, then transition to the next state.
   */
  void FOC::onPark()
  {
    // ParkControl *const pCtrl        = &mState.motorCtl.park;
    // const uint32_t     current_time = Chimera::millis();

    // /*-------------------------------------------------------------------------
    // Has the alignment sequence run its course? Switch to the ramp phase.
    // -------------------------------------------------------------------------*/
    // if ( ( current_time - pCtrl->startTime_ms ) > pCtrl->alignTime_ms )
    // {
    //   this->receive( MsgRamp() );
    //   RT_DBG_ASSERT( this->currentMode() == ModeId::ENGAGED_RAMP );
    // }

    // /*-------------------------------------------------------------------------
    // Otherwise, update the alignment controller
    // -------------------------------------------------------------------------*/
    // pCtrl->activeComState = 1;

    // if ( ( current_time - pCtrl->lastUpdate_ms ) > pCtrl->modulation_dt_ms )
    // {
    //   pCtrl->outputEnabled = !pCtrl->outputEnabled;
    //   pCtrl->lastUpdate_ms = current_time;
    // }
  }


  void FOC::onRamp()
  {
    // RampControl *const pCtrl = &mState.motorCtl.ramp;

    // /*-------------------------------------------------------------------------
    // Switch to the RUN controller if we've hit our target RPM.
    // -------------------------------------------------------------------------*/
    // if ( pCtrl->targetRPM >= pCtrl->finalRPM )
    // {
    //   // pCtrl->phaseDutyCycle[ 0 ] = 20.0f;
    //   // pCtrl->phaseDutyCycle[ 1 ] = 20.0f;
    //   // pCtrl->phaseDutyCycle[ 2 ] = 20.0f;
    //   // this->receive( MsgRun() );
    //   // RT_DBG_ASSERT( this->currentMode() == ModeId::ENGAGED_RUN );
    //   return;
    // }

    // /*-------------------------------------------------------------------------
    // Otherwise update the controller reference
    // -------------------------------------------------------------------------*/
    // pCtrl->targetRPM += pCtrl->rampRate;
  }


  void FOC::onRun()
  {
  }

}    // namespace Orbit::Control


/*
Notes and Ideas:

- Dynamically adjust the PWM slicing frequency based on the system input voltage. This drives the theoretical maximum
  performance of the current controller. Can optimize for power dissipation or complete sine wave approximation. Could use
  sine wave control at lower speeds and then move into coarser control as speed increases.

    - System kV estimator (drives predictions of ISR limits)
    - Programmable kV "hint"
    - Measure runtime ISR speed and adjust sine approximation based on real performance limits.

  Ideally this will extend the control range of the ESC to very high kV/battery systems while also achieving the efficiency
  of sinusoidal drives.

    - Limiting factors will be overall system bandwidth. Acceleration may be a problem.
*/