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
#include <cmath>
#include <Chimera/adc>
#include <src/control/foc_driver.hpp>
#include <src/config/bsp/board_map.hpp>

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


  int FOC::initialize( const FOCConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Validate the configuration
    -------------------------------------------------------------------------*/
    for ( auto &txfrFunc : cfg.txfrFuncs )
    {
      RT_DBG_ASSERT( txfrFunc != nullptr );
    }

    /*-------------------------------------------------------------------------
    Initialize the FOC state
    -------------------------------------------------------------------------*/
    mPrvState.clear();
    mConfig.clear();

    mConfig = cfg;

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
    mPrvState.adcBuffer[ ADC_CH_MOTOR_PHASE_A_CURRENT ].dcOffset = mADCDriver->toVoltage( sample );

    sample = mADCDriver->sampleChannel( Orbit::IO::Analog::adcPhaseB );
    mPrvState.adcBuffer[ ADC_CH_MOTOR_PHASE_B_CURRENT ].dcOffset = mADCDriver->toVoltage( sample );

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

    return 0;
  }


  void FOC::lastSensorData( ADCSensorBuffer &data )
  {
    data = mPrvState.adcBuffer;
  }


  void FOC::dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr )
  {
    /*-------------------------------------------------------------------------
    Convert the ADC data to measured values
    -------------------------------------------------------------------------*/
    static constexpr float COUNTS_TO_VOLTS = 3.3f / 4096.0f; // Vref = 3.3V, 12-bit ADC

    for( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      mPrvState.adcBuffer[ i ].measured  = static_cast<float>( isr.samples[ i ] ) * COUNTS_TO_VOLTS;
      mPrvState.adcBuffer[ i ].converted = mConfig.txfrFuncs[ i ]( mPrvState.adcBuffer[ i ].measured );
    }

    /*-------------------------------------------------------------------------
    Calculate flux linkage
    -------------------------------------------------------------------------*/


    /*-------------------------------------------------------------------------
    Update commutation for testing
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
  }


  void FOC::timer_isr_speed_controller()
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