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
#include <Thor/lld/interface/inc/timer>
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

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
#include "SEGGER_SYSVIEW.h"
#endif

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
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<FOC, &FOC::dma_isr_current_controller>( *this );

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
    pwm_cfg.pwmFrequency        = 24'000.0f;

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
    mState.motorController.run.speedRefRad = ref;
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


  void FOC::dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr )
  {
    SEGGER_SYSVIEW_RecordEnterISR();

    /*-------------------------------------------------------------------------
    Grab a few pieces of information
    -------------------------------------------------------------------------*/
    const ModeId_t current_mode = static_cast<ModeId_t>( this->get_state_id() );
    const uint32_t timestamp_us = Chimera::micros();

    /*-------------------------------------------------------------------------
    Convert the ADC data to measured values
    -------------------------------------------------------------------------*/
    static constexpr float COUNTS_TO_VOLTS = 3.3f / 4096.0f;    // Vref = 3.3V, 12-bit ADC

    for ( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      mState.adcBuffer[ i ].measured     = static_cast<float>( isr.samples[ i ] ) * COUNTS_TO_VOLTS;
      mState.adcBuffer[ i ].converted    = mConfig.txfrFuncs[ i ]( mState.adcBuffer[ i ].measured );
      mState.adcBuffer[ i ].sampleTimeUs = timestamp_us;
    }

    /*-------------------------------------------------------------------------
    Estimate the rotor position
    -------------------------------------------------------------------------*/
    if ( ( current_mode == ModeId::ENGAGED_RUN ) || ( current_mode == ModeId::ENGAGED_RAMP ) )
    {
      /*-----------------------------------------------------------------------
      Move the sampled phase currents through the Clarke-Park transform
      -----------------------------------------------------------------------*/
      const auto clarke = Math::clarke_transform( mState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].converted,
                                                  mState.adcBuffer[ ADC_CH_MOTOR_PHASE_B_CURRENT ].converted );

      const auto park = Math::park_transform( clarke, mState.motorController.posEstRad );

      /*-----------------------------------------------------------------------
      Do the estimation
      -----------------------------------------------------------------------*/
      /* Update the motor state data from previous calculations */
      mState.motorController.Iq  = park.q;
      mState.motorController.Id  = park.d;
      mState.motorController.Vdd = mState.adcBuffer[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].converted;

      /* Step the EMF observer */
      const float dt = US_TO_SEC( timestamp_us - mState.emfObserver.last_update_us );
      stepEMFObserver( dt );
      mState.emfObserver.last_update_us = timestamp_us;

      /* Calculate the estimated rotor position (Eq. 18) */
      mState.motorController.posEstRad = Math::fast_atan2_with_norm( -mState.emfObserver.Ed_est, mState.emfObserver.Eq_est );
      if ( mState.motorController.velEstRad < 0.0f )
      {
        mState.motorController.posEstRad += Math::M_PI_F;
      }
    }

    /*-------------------------------------------------------------------------
    Based on the current state, decide how to drive the output stage
    -------------------------------------------------------------------------*/
    if ( current_mode == ModeId::ENGAGED_RUN ) {}
    else if ( current_mode == ModeId::ENGAGED_RAMP ) {}
    else if ( current_mode == ModeId::ENGAGED_PARK )
    {
      if ( mState.motorController.park.outputEnabled )
      {
        mTimerDriver.setPhaseDutyCycle( mState.motorController.park.phaseDutyCycle[ 0 ],
                                        mState.motorController.park.phaseDutyCycle[ 1 ],
                                        mState.motorController.park.phaseDutyCycle[ 2 ] );

        mTimerDriver.setForwardCommState( mState.motorController.park.activeComState );
      }
      else
      {
        mTimerDriver.setForwardCommState( 0 );
      }
    }
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

    /*-------------------------------------------------------------------------
    Static commutation for testing
    -------------------------------------------------------------------------*/
    // static uint32_t cycle_count = 0;
    // static uint32_t phase       = 0;

    // cycle_count++;
    // if ( cycle_count >= 100 )
    // {
    //   cycle_count = 0;
    //   mTimerDriver.setForwardCommState( phase );

    //   phase++;
    //   if ( phase >= 6 )
    //   {
    //     phase = 0;
    //   }
    // }

    SEGGER_SYSVIEW_RecordExitISR();
  }


  void FOC::timer_isr_speed_controller()
  {
    mSpeedCtrlTrigger.ackISR();

    /*-------------------------------------------------------------------------
    Don't run the speed controller loop if not in closed loop control
    -------------------------------------------------------------------------*/
    if ( this->get_state_id() != EnumValue( ModeId::ENGAGED_RUN ) )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Update estimate of the rotor speed
    -------------------------------------------------------------------------*/
    const size_t timestamp = Chimera::micros();
    const float  dTheta    = mState.motorController.posEstRad - mState.speedEstimator.pos_est_prev;
    const float  wdt       = US_TO_SEC( timestamp - mState.speedEstimator.last_update_us );

    mState.motorController.velEstRad = dTheta / wdt;

    /*-----------------------------------------------------------------------
    Update tracking variables
    -----------------------------------------------------------------------*/
    mState.speedEstimator.pos_est_prev   = mState.motorController.posEstRad;
    mState.speedEstimator.last_update_us = timestamp;

    // Reset the timer interrupt flag...
  }


  void FOC::stepEMFObserver( const float dt )
  {
    /*-------------------------------------------------------------------------
    Alias equation variables to make everything easier to read
    -------------------------------------------------------------------------*/
    const float d  = mState.emfObserver.d;
    const float Ls = mState.motorParams.Ls;
    const float Rs = mState.motorParams.Rs;
    const float Wr = mState.motorController.velEstRad;
    const float Iq = mState.motorController.Iq;
    const float Id = mState.motorController.Id;
    const float Vq = mState.motorController.Vq;
    const float Vd = mState.motorController.Vd;

    /*-------------------------------------------------------------------------
    Update the D-Q axis observer state space model (Eq. 11)
    -------------------------------------------------------------------------*/
    const float q_tmp = ( ( Rs + ( d * Ls ) ) * Iq ) - Vq;
    const float d_tmp = ( ( Rs + ( d * Ls ) ) * Id ) - Vd;

    mState.emfObserver.z1_dot = ( d * dt * mState.emfObserver.z1 ) + dt * ( ( d * q_tmp ) - ( Wr * d_tmp ) );
    mState.emfObserver.z2_dot = ( d * dt * mState.emfObserver.z2 ) + dt * ( ( Wr * q_tmp ) + ( d * d_tmp ) );

    /*-------------------------------------------------------------------------
    Calculate the estimated EMF (Eq. 12)
    -------------------------------------------------------------------------*/
    mState.emfObserver.Eq_est = mState.emfObserver.z1_dot + ( d * Ls * Iq ) - ( Ls * Wr * Id );
    mState.emfObserver.Ed_est = mState.emfObserver.z2_dot + ( Ls * Wr * Iq ) + ( d * Ls * Id );

    /*-------------------------------------------------------------------------
    Update the EMF observer state. Note that Z1 dot already has the sample
    time baked in from Equation 11.
    -------------------------------------------------------------------------*/
    mState.emfObserver.z1 = mState.emfObserver.z1_dot;
    mState.emfObserver.z2 = mState.emfObserver.z2_dot;
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
    ParkControl *const pCtrl        = &mState.motorController.park;
    const uint32_t     current_time = Chimera::millis();

    /*-------------------------------------------------------------------------
    Has the alignment sequence run its course? Switch to the ramp phase.
    -------------------------------------------------------------------------*/
    if( ( current_time - pCtrl->startTime_ms ) > pCtrl->alignTime_ms )
    {
      this->receive( MsgRamp() );
      RT_DBG_ASSERT( this->currentMode() == ModeId::ENGAGED_RAMP );
    }

    /*-------------------------------------------------------------------------
    Otherwise, update the alignment controller
    -------------------------------------------------------------------------*/
    pCtrl->activeComState = 1;

    if( ( current_time - pCtrl->lastUpdate_ms ) > pCtrl->modulation_dt_ms )
    {
      pCtrl->outputEnabled = !pCtrl->outputEnabled;
      pCtrl->lastUpdate_ms = current_time;
    }
  }


  void FOC::onRamp()
  {
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