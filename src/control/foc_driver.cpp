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
#include <Thor/lld/interface/inc/timer>

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
#include "SEGGER_SYSVIEW.h"
#endif

extern volatile bool isr_foc_flag;

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  FOC FOCDriver;

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  FOC::FOC()
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

    mConfig = cfg;
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

    sample = mADCDriver->sampleChannel( Orbit::IO::Analog::adcPhaseA );
    mState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].dcOffset = mADCDriver->toVoltage( sample );

    sample = mADCDriver->sampleChannel( Orbit::IO::Analog::adcPhaseB );
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
    /* Fill the state array with the class objects */
    mFSMStateArray.fill( nullptr );
    mFSMStateArray[ ModeId::IDLE ]         = &mIdleState;
    mFSMStateArray[ ModeId::ARMED ]        = &mArmedState;
    mFSMStateArray[ ModeId::FAULT ]        = &mFaultState;
    mFSMStateArray[ ModeId::ENGAGED_PARK ] = &mParkState;
    mFSMStateArray[ ModeId::ENGAGED_RAMP ] = &mRampState;
    mFSMStateArray[ ModeId::ENGAGED_RUN ]  = &mRunState;

    /* Initialize the FSM. First state will be ModeId::IDLE. */
    mFSM.attachControllerData( &mState );
    mFSM.set_states( mFSMStateArray.data(), mFSMStateArray.size() );
    mFSM.start();

    return 0;
  }


  int FOC::arm()
  {
    mFSM.receive( MsgArm() );
    return this->currentMode() == ModeId::ARMED ? 0 : -1;
  }


  int FOC::disarm()
  {
    mFSM.receive( MsgDisarm() );
    return 0;
  }


  int FOC::engage()
  {
    mFSM.receive( MsgAlign() );
    return 0;
  }


  int FOC::disengage()
  {
    mFSM.receive( MsgDisengage() );
    return 0;
  }


  int FOC::setSpeedRef( const float ref )
  {
    return 0;
  }


  int FOC::emergencyStop()
  {
    mFSM.receive( MsgEmergencyHalt() );
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
    return mFSM.get_state_id();
  }


  void FOC::dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr )
  {
    SEGGER_SYSVIEW_RecordEnterISR();

    /*-------------------------------------------------------------------------
    Convert the ADC data to measured values
    -------------------------------------------------------------------------*/
    static constexpr float COUNTS_TO_VOLTS = 3.3f / 4096.0f; // Vref = 3.3V, 12-bit ADC

    const uint32_t timestamp = Chimera::micros();
    for( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      mState.adcBuffer[ i ].measured     = static_cast<float>( isr.samples[ i ] ) * COUNTS_TO_VOLTS;
      mState.adcBuffer[ i ].converted    = mConfig.txfrFuncs[ i ]( mState.adcBuffer[ i ].measured );
      mState.adcBuffer[ i ].sampleTimeUs = timestamp;
    }

    /*-------------------------------------------------------------------------
    Move the sampled phase currents through the Clarke-Park transform
    -------------------------------------------------------------------------*/
    const auto clarke = Math::clarke_transform( mState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].converted,
                                                mState.adcBuffer[ ADC_CH_MOTOR_PHASE_B_CURRENT ].converted );

    const auto park = Math::park_transform( clarke, mState.motorState.posEstRad );

    /*-------------------------------------------------------------------------
    Calculate the estimated rotor position
    -------------------------------------------------------------------------*/
    /* Update the motor state data from previous calculations */
    mState.motorState.Iq  = park.q;
    mState.motorState.Id  = park.d;
    mState.motorState.Vdd = mState.adcBuffer[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].converted;

    /* Step the EMF observer */
    const float dt = US_TO_SEC( timestamp - mState.emfObserver.last_update_us );
    stepEMFObserver( dt );
    mState.emfObserver.last_update_us = timestamp;

    /* Calculate the estimated rotor position (Eq. 18) */
    mState.motorState.posEstRad = Math::fast_atan2_with_norm( -mState.emfObserver.Ed_est, mState.emfObserver.Eq_est );
    if ( mState.motorState.velEstRad < 0.0f )
    {
      mState.motorState.posEstRad += Math::M_PI_F;
    }

    /*-------------------------------------------------------------------------
    Update the commanded controller output states
    -------------------------------------------------------------------------*/
    const Math::ParkSpace park_space{.d = mState.motorState.Vd, .q = mState.motorState.Vq };

    auto invPark = Math::inverse_park_transform( park_space, mState.motorState.posEstRad );
    float a, b, c;

    Math::inverse_clarke_transform( invPark, &a, &b, &c );

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
    static uint32_t cycle_count = 0;
    static uint32_t phase       = 0;

    cycle_count++;
    if ( cycle_count >= 100 )
    {
      cycle_count = 0;
      mTimerDriver.setForwardCommState( phase );

      phase++;
      if ( phase >= 6 )
      {
        phase = 0;
      }
    }

    SEGGER_SYSVIEW_RecordExitISR();
  }


  void FOC::timer_isr_speed_controller()
  {
    mSpeedCtrlTrigger.ackISR();

    /*-------------------------------------------------------------------------
    Update estimate of the rotor speed
    -------------------------------------------------------------------------*/
    const size_t timestamp = Chimera::micros();
    const float  dTheta    = mState.motorState.posEstRad - mState.speedEstimator.pos_est_prev;
    const float  wdt       = US_TO_SEC( timestamp - mState.speedEstimator.last_update_us );

    mState.motorState.velEstRad = dTheta / wdt;

    /*-----------------------------------------------------------------------
    Update tracking variables
    -----------------------------------------------------------------------*/
    mState.speedEstimator.pos_est_prev   = mState.motorState.posEstRad;
    mState.speedEstimator.last_update_us = timestamp;


    isr_foc_flag = true;
    // Reset the timer interrupt flag...
  }


  void FOC::stepEMFObserver( const float dt )
  {
    /*-------------------------------------------------------------------------
    Alias equation variables to make everything easier to read
    -------------------------------------------------------------------------*/
    const float d = mState.emfObserver.d;
    const float Ls = mState.motorParams.Ls;
    const float Rs = mState.motorParams.Rs;
    const float Wr = mState.motorState.velEstRad;
    const float Iq = mState.motorState.Iq;
    const float Id = mState.motorState.Id;
    const float Vq = mState.motorState.Vq;
    const float Vd = mState.motorState.Vd;

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