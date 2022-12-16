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
  FOC::FOC() : mInitialized( false ), fsm( MOTOR_STATE_ROUTER_ID )
  {
  }

  FOC::~FOC()
  {
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

    ctl->Dpid.SetPoint    = 0.0f;
    ctl->Dpid.OutMinLimit = 0.0f;
    ctl->Dpid.OutMaxLimit = 1.0f;
    ctl->DFIR = Math::FIR<float, 15>( { -0.00124568841F, 0.00147019443F, 0.0123328818F, 0.00110139197F, -0.0499843247F,
                                        -0.0350326933F, 0.164325342F, 0.407320708F, 0.407320708F, 0.164325342F, -0.0350326933F,
                                        -0.0499843247F, 0.00110139197F, 0.0123328818F, 0.00147019443F, -0.00124568841F } );
    ctl->DFIR.initialize();

    ctl->Qpid.SetPoint    = 0.0f;
    ctl->Qpid.OutMinLimit = 0.0f;
    ctl->Qpid.OutMaxLimit = 1.0f;
    ctl->QFIR = Math::FIR<float, 15>( { -0.00124568841F, 0.00147019443F, 0.0123328818F, 0.00110139197F, -0.0499843247F,
                                        -0.0350326933F, 0.164325342F, 0.407320708F, 0.407320708F, 0.164325342F, -0.0350326933F,
                                        -0.0499843247F, 0.00110139197F, 0.0123328818F, 0.00147019443F, -0.00124568841F } );
    ctl->QFIR.initialize();
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
    pwm_cfg.pwmFrequency        = Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ;

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
    mFSMStateArray[ ModeId::IDLE ]    = new State::Idle();
    mFSMStateArray[ ModeId::ARMED ]   = new State::Armed();
    mFSMStateArray[ ModeId::FAULT ]   = new State::Fault();
    mFSMStateArray[ ModeId::ENGAGED ] = new State::Engaged();

    /* Initialize the FSM. First state will be ModeId::IDLE. */
    this->set_states( mFSMStateArray.data(), mFSMStateArray.size() );
    this->start();

    mInitialized = true;
    return 0;
  }


  void FOC::calibrate()
  {
    /*-------------------------------------------------------------------------
    Ensure the motor is off for calibration
    -------------------------------------------------------------------------*/
    sendSystemEvent( EventId::EMERGENCY_HALT );
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
    mState.motorCtl.isrCtlActive = true;
  }


  int FOC::sendSystemEvent( const EventId_t event )
  {
    /*-------------------------------------------------------------------------
    Make sure the state machine controller is ready
    -------------------------------------------------------------------------*/
    if( !mInitialized )
    {
      return -1;
    }

    /*-------------------------------------------------------------------------
    Send the correct message into the state machine
    -------------------------------------------------------------------------*/
    switch( event )
    {
      case EventId::EMERGENCY_HALT:
        this->receive( MsgEmergencyHalt() );
        break;

      case EventId::ARM:
        this->receive( MsgArm() );
        break;

      case EventId::DISARM:
        this->receive( MsgDisarm() );
        break;

      case EventId::ENGAGE:
        this->receive( MsgEngage() );
        break;

      case EventId::DISENGAGE:
        this->receive( MsgDisengage() );
        break;

      case EventId::FAULT:
        this->receive( MsgFault() );
        break;

      default:
        return -1;
        break;
    }

    return 0;
  }


  int FOC::setSpeedRef( const float ref )
  {
    return 0;
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
    if( mInitialized )
    {
      return this->get_state_id();
    }
    else
    {
      return ModeId::NUM_STATES;
    }
  }


  void FOC::logUnhandledMessage( const etl::imessage &msg )
  {
    LOG_WARN( "%s message not handled from state %s\r\n", getMessageString( msg.get_message_id() ).data(),
              getModeString( get_state_id() ).data() );
  }


  void FOC::driveTestSignal( const uint8_t commCycle, const float dutyCycle )
  {
    /*-------------------------------------------------------------------------
    Ensure we're in the ARMED state before attempting anything. This guarantees
    our protections are running but the output is disabled.
    -------------------------------------------------------------------------*/
    if( currentMode() != ModeId::ARMED )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Apply the test signal
    -------------------------------------------------------------------------*/
    mTimerDriver.setPhaseDutyCycle( dutyCycle, dutyCycle, dutyCycle );
    mTimerDriver.setForwardCommState( commCycle );
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

    /* Low-pass filter the measured currents */
    mState.motorCtl.Idf = mState.motorCtl.DFIR.step( mState.motorCtl.Idm );
    mState.motorCtl.Iqf = mState.motorCtl.QFIR.step( mState.motorCtl.Iqm );

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

    // mState.motorCtl.svpwm_a_duty = ( a + 0.5f ) * 100.0f;
    // mState.motorCtl.svpwm_b_duty = ( b + 0.5f ) * 100.0f;
    // mState.motorCtl.svpwm_c_duty = ( c + 0.5f ) * 100.0f;
    // mState.motorCtl.svpwm_comm   = sector;

    static uint32_t counter = 0;
    mState.motorCtl.svpwm_a_duty = 10.0f;
    mState.motorCtl.svpwm_b_duty = 10.0f;
    mState.motorCtl.svpwm_c_duty = 10.0f;

    counter++;
    if( counter > 10 )
    {
      counter = 0;
      mState.motorCtl.svpwm_comm++;
      if( mState.motorCtl.svpwm_comm >= 7 )
      {
        mState.motorCtl.svpwm_comm = 1;
      }
    }


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