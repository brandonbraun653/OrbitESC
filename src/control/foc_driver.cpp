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
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/utility.hpp>
#include <src/monitor/orbit_monitors.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/interface/inc/timer>

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* SEGGER_SYS_VIEW */
#endif /* EMBEDDED */


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
    Initialize eigenvalues for the current controller
    -------------------------------------------------------------------------*/
    mState.emfObserver.d = ( -motorParams.Rs / motorParams.Ls ) * 5.0f;


    ControllerData* ctl = &mState.motorCtl;

    ctl->clear();

    ctl->Dpid.SetPoint = 0.0f;
    ctl->Dpid.OutMinLimit = 0.0f;
    ctl->Dpid.OutMaxLimit = 1.0f;

    ctl->Qpid.SetPoint = 0.0f;
    ctl->Qpid.OutMinLimit = 0.0f;
    ctl->Qpid.OutMaxLimit = 1.0f;
    ctl->Qpid.setTunings( 5.0f, 2.0f, 0.3f, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );


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
    trig_cfg.trigFreq               = Orbit::Data::DFLT_SPEED_CTL_UPDT_FREQ_HZ;
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


  void FOC::calibrate()
  {
    /*-------------------------------------------------------------------------
    Ensure the motor is off for calibration
    -------------------------------------------------------------------------*/
    int stopped = this->emergencyStop();
    RT_DBG_ASSERT( stopped == 0 );
    Chimera::delayMilliseconds( 100 );

    /*-------------------------------------------------------------------------
    Measure the DC offset of the motor phase current sensors
    -------------------------------------------------------------------------*/
    LOG_TRACE( "Calibrating phase current sensors\r\n" );
    for( int idx = ADC_CH_MOTOR_PHASE_A_CURRENT; idx <= ADC_CH_MOTOR_PHASE_C_CURRENT; idx++ )
    {
      float samples = 0.0f;
      float pIxAvg  = 0.0f;

      while ( samples < 25.0f )
      {
        samples++;
        pIxAvg += mState.adcBuffer[ idx ].measured;
        Chimera::delayMilliseconds( 5 );
      }

      mState.adcBuffer[ idx ].dcOffset = ( pIxAvg / samples );
    }
    LOG_TRACE( "Done\r\n" );
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
    // this->receive( MsgAlign() );

    // auto mode = this->currentMode();
    // return ( ( mode == ModeId::ENGAGED_PARK ) || ( mode == ModeId::ENGAGED_RAMP ) || ( mode == ModeId::ENGAGED_RUN ) ) ? 0 : -1;


    mTimerDriver.disableOutput();
    mTimerDriver.setForwardCommState( 0 );
    mTimerDriver.enableOutput();

    mState.motorCtl.isrCtlActive = true;
    mState.motorCtl.Iqr = 0.001;
    mState.motorCtl.posEst = 0.0f;
    mState.motorCtl.spdEst = 0.0f;
    mState.motorCtl.IqrInt.reset();
    mState.motorCtl.SpdInt.reset();

    return 0;
  }


  int FOC::disengage()
  {
    this->receive( MsgDisengage() );
    return ( this->currentMode() == ModeId::ARMED ) ? 0 : -1;
  }


  int FOC::setSpeedRef( const float ref )
  {
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
    using namespace Orbit::Monitor;

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    /*-------------------------------------------------------------------------
    Process the raw ADC data
    -------------------------------------------------------------------------*/
    const uint32_t timestamp_us    = Chimera::micros();
    const float    f_timestamp_us  = static_cast<float>( timestamp_us );
    const float    counts_to_volts = isr.vref / isr.resolution;

    for ( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      /*-----------------------------------------------------------------------
      Convert the ADC data to measured values

      TODO: Use different samples for each channel depending on which channel
      has the low time width. Fig. 17-10 InstaSPIN-FOC. Should improve ADC
      technique with 3- shunt
      -----------------------------------------------------------------------*/
      mState.adcBuffer[ i ].measured     = static_cast<float>( isr.samples[ i ] ) * counts_to_volts;
      mState.adcBuffer[ i ].calibrated   = mState.adcBuffer[ i ].measured - mState.adcBuffer[ i ].dcOffset;
      mState.adcBuffer[ i ].converted    = mConfig.txfrFuncs[ i ]( mState.adcBuffer[ i ].calibrated );
      mState.adcBuffer[ i ].sampleTimeUs = timestamp_us;

      /*-----------------------------------------------------------------------
      Update the monitor for this channel

      // TODO BMB: Consider using the analog watchdog
      -----------------------------------------------------------------------*/
      IAnalogMonitor *monitor = MonitorArray[ i ];
      monitor->update( mState.adcBuffer[ i ].converted, mState.adcBuffer[ i ].sampleTimeUs );
      if( monitor->tripped() == TripState::NOT_TRIPPED )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Tripped! Take action and move the motor to a safe state.
      -----------------------------------------------------------------------*/
      mState.motorCtl.isrCtlActive = false;
      this->emergencyStop();
      monitor->setEngageState( Orbit::Monitor::EngageState::INACTIVE );
      Orbit::LED::setChannel( Orbit::LED::Channel::FAULT );
    }

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    if ( !mState.motorCtl.isrCtlActive )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Apply the latest motor control command
    -------------------------------------------------------------------------*/
    mTimerDriver.setPhaseDutyCycle( mState.motorCtl.svpwm_a_duty, mState.motorCtl.svpwm_b_duty, mState.motorCtl.svpwm_c_duty );
    mTimerDriver.setForwardCommState( mState.motorCtl.svpwm_comm );

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


  void FOC::timer_isr_speed_controller()
  {
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
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
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    mSpeedCtrlTrigger.ackISR();
    if ( !mState.motorCtl.isrCtlActive )
    {
      return;
    }

    /*-----------------------------------------------------------------------
    Move the sampled phase currents through the Clarke-Park transform

    TODO: Use 3-phase current measurements due to slew rate improvement.s
    -----------------------------------------------------------------------*/
    const auto clarke = Math::clarke_transform( mState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].converted,
                                                mState.adcBuffer[ ADC_CH_MOTOR_PHASE_B_CURRENT ].converted );

    const auto park = Math::park_transform( clarke, mState.motorCtl.posEst );

    /*-----------------------------------------------------------------------
    Run the control loop
    -----------------------------------------------------------------------*/
    /* Update the motor state data from previous calculations */
    mState.motorCtl.Iqm = park.q;
    mState.motorCtl.Idm = park.d;
    mState.motorCtl.Vdd = mState.adcBuffer[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].converted;

    /* Filter the measured currents */
    // TODO BMB: Run this through the LPF once created
    mState.motorCtl.Idf = mState.motorCtl.Idm;
    mState.motorCtl.Iqf = mState.motorCtl.Iqm;

    /* Step the PID controllers */
    mState.motorCtl.Dpid.run( mState.motorCtl.Idr - mState.motorCtl.Idf );
    mState.motorCtl.Vdr = mState.motorCtl.Dpid.Output;

    mState.motorCtl.Qpid.run( mState.motorCtl.Iqr - mState.motorCtl.Iqf );
    mState.motorCtl.Vqr = mState.motorCtl.Qpid.Output;

    /* Open loop control motor speed and position estimates */
    mState.motorCtl.spdEst = mState.motorCtl.IqrInt.step( mState.motorCtl.Iqr, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );
    mState.motorCtl.posEst = mState.motorCtl.SpdInt.step( mState.motorCtl.spdEst, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );

    /* Naive wrapping of position */
    if( mState.motorCtl.posEst > Math::M_2PI_F )
    {
      mState.motorCtl.posEst -= Math::M_2PI_F;
    }
    else if( mState.motorCtl.posEst < 0.0f )
    {
      mState.motorCtl.posEst += Math::M_2PI_F;
    }

    /*-------------------------------------------------------------------------
    Update the commanded controller output states
    -------------------------------------------------------------------------*/
    float                 a, b, c;
    const Math::ParkSpace park_space{ .d = mState.motorCtl.Vdr, .q = mState.motorCtl.Vqr };
    auto                  invPark = Math::inverse_park_transform( park_space, mState.motorCtl.posEst );

    Math::inverse_clarke_transform( invPark, &a, &b, &c );

    /*-------------------------------------------------------------------------
    Use the position estimate to determine the next commutation state. Split
    the unit circle into 6 sectors and select the appropriate one to commutate.

    https://math.stackexchange.com/a/206662/435793
    Solve for N => Theta * 3 / pi
    -------------------------------------------------------------------------*/
    static constexpr float SECTOR_CONV_FACTOR = 3.0f / Math::M_PI_F;
    uint8_t                sector = 1 + static_cast<uint8_t>( SECTOR_CONV_FACTOR * mState.motorCtl.posEst );

    mState.motorCtl.svpwm_a_duty = a;
    mState.motorCtl.svpwm_b_duty = b;
    mState.motorCtl.svpwm_c_duty = c;
    mState.motorCtl.svpwm_comm   = sector;


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

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
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
    //   pCtrl->cycleRef   = Utility::comCycleCount( Data::DFLT_STATOR_PWM_FREQ_HZ, Data::DFLT_ROTOR_NUM_POLES, pCtrl->targetRPM
    //   );

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