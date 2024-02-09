/******************************************************************************
 *  File Name:
 *    orbit_motor_sense.cpp
 *
 *  Description:
 *    Hardware driver for the motor sensor subsystem. This module is
 *    responsible for reading the motor phase currents and voltages and calling
 *    a handler for processing the results.
 *
 *  Notes:
 *    The ADC is configured to run in DMA mode and trigger off of a timer that
 *    is synchronized with the motor power stage PWM timer. This ensures that
 *    the ADC samples are taken at the same time as the power stage is active.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/core/hw/orbit_timer.hpp>
#include <src/control/foc_math.hpp>

namespace Orbit::Motor::Sense
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct SenseControlBlock
  {
    float                      adc_vref;                   /**< ADC reference voltage */
    float                      adc_vres;                   /**< ADC resolution in counts/volt */
    float                      rawData[ CHANNEL_COUNT ];   /**< Raw measured voltage */
    float                      calOffset[ CHANNEL_COUNT ]; /**< Calibration offset voltage */
    float                      calData[ CHANNEL_COUNT ];   /**< Calibrated voltage */
    SenseData                  siData;                     /**< Data interpreted as SI units */
    SenseCallback              callback;                   /**< User adc complete callback */
    Chimera::GPIO::Driver_rPtr dbg_pin;                    /**< Debug output pin for timing */
  };

  struct IPhaseCal
  {
    float dcOffset; /**< Averaged DC offset when inputs are shorted */
    float floor;    /**< Minimum value seen */
    float ceiling;  /**< Maximum value seen */
  };

  using IPhaseCalArray = etl::array<IPhaseCal, 3>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile SenseControlBlock     s_ctl_blk;
  static Chimera::Timer::Trigger::Slave s_motor_sense_timer;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Converts an ADC count to a voltage
   *
   * @param counts  ADC count to convert
   * @return float  Voltage in volts
   */
  static inline float counts_to_voltage( const uint16_t counts )
  {
    return ( static_cast<float>( counts ) * s_ctl_blk.adc_vref ) / s_ctl_blk.adc_vres;
  }


  /**
   * @brief Gets the current time in seconds with microsecond precision
   * @return float  Time in seconds
   */
  static inline float current_time_sec()
  {
    return static_cast<float>( CortexM4::SYSTick::getMicroseconds() ) / 1e6f;
  }


  /**
   * @brief Computes phase current from the ADC voltage reading
   *
   * @param vin     ADC voltage reading
   * @return float  Phase current in amps
   */
  static inline float compute_phase_current( const float vin )
  {
    /*-------------------------------------------------------------------------
    OrbitESC v3 circuit parameters
    -------------------------------------------------------------------------*/
    constexpr float ISHUNT_AMP_GAIN = 10.0f; /**< Configured gain of all amplifiers */
    constexpr float RSHUNT_OHM      = 0.01f; /**< Low-side shunt resistor */

    /*-------------------------------------------------------------------------
    Calculate backwards from the ADC reading to what the real current is. The
    op-amp is configured as an inverting amplifier, so a positive current will
    result in a negative voltage delta. The current is then calculated by
    dividing the voltage by the shunt resistor and the op-amp gain.
    -------------------------------------------------------------------------*/
    const float raw = -1.0f * ( vin / ( ISHUNT_AMP_GAIN * RSHUNT_OHM ) );
    return raw;
  }


  /**
   * @brief Computes phase voltage from the ADC voltage reading
   *
   * @param vin     ADC voltage reading
   * @return float  Phase voltage in volts
   */
  static inline float compute_phase_voltage( const float vin )
  {
    /*-------------------------------------------------------------------------
    OrbitESC v3 circuit parameters
    -------------------------------------------------------------------------*/
    constexpr float PHASE_V_SENSE_R1          = 39'000.0f;
    constexpr float PHASE_V_SENSE_R2          = 5'100.0f;
    constexpr float PHASE_V_DIV_RATIO         = PHASE_V_SENSE_R2 / ( PHASE_V_SENSE_R1 + PHASE_V_SENSE_R2 );
    constexpr float ADC_TO_PHASE_V_CONV_CONST = 1.0f / PHASE_V_DIV_RATIO;

    /*-------------------------------------------------------------------------
    Use voltage divider ratio to convert ADC voltage to phase voltage
    -------------------------------------------------------------------------*/
    return vin * ADC_TO_PHASE_V_CONV_CONST;
  }


  /**
   * @brief Callback to handle results of ADC conversions
   *
   * @param isrData Results of the ADC conversion
   * @return void
   */
  static void isr_on_motor_sense_adc_conversion_complete( const Chimera::ADC::InterruptDetail &isr )
  {
    using namespace Orbit::Control::Math;

    /*-------------------------------------------------------------------------
    Set the debug pin high to start measuring ISR execution time. Should be
    set low again via the user callback.
    -------------------------------------------------------------------------*/
    s_ctl_blk.dbg_pin->setState( Chimera::GPIO::State::HIGH );

    /*-------------------------------------------------------------------------
    Update the sense data cache
    -------------------------------------------------------------------------*/
    s_ctl_blk.siData.timestamp = current_time_sec();
    s_ctl_blk.adc_vref         = isr.vref;
    s_ctl_blk.adc_vres         = isr.resolution;

    for ( auto i = 0; ( i < isr.num_samples ) && ( i < CHANNEL_COUNT ); i++ )
    {
      s_ctl_blk.rawData[ i ] = counts_to_voltage( isr.samples[ i ] );
      s_ctl_blk.calData[ i ] = s_ctl_blk.rawData[ i ] - s_ctl_blk.calOffset[ i ];
    }

    /*-------------------------------------------------------------------------
    Translate raw measurements into representative SI units
    -------------------------------------------------------------------------*/
    s_ctl_blk.siData.channel[ CHANNEL_PHASE_A_CURRENT ] = compute_phase_current( s_ctl_blk.calData[ CHANNEL_PHASE_A_CURRENT ] );
    s_ctl_blk.siData.channel[ CHANNEL_PHASE_B_CURRENT ] = compute_phase_current( s_ctl_blk.calData[ CHANNEL_PHASE_B_CURRENT ] );
    s_ctl_blk.siData.channel[ CHANNEL_PHASE_C_CURRENT ] = compute_phase_current( s_ctl_blk.calData[ CHANNEL_PHASE_C_CURRENT ] );
    s_ctl_blk.siData.channel[ CHANNEL_PHASE_A_VOLTAGE ] = compute_phase_voltage( s_ctl_blk.calData[ CHANNEL_PHASE_A_VOLTAGE ] );
    s_ctl_blk.siData.channel[ CHANNEL_PHASE_B_VOLTAGE ] = compute_phase_voltage( s_ctl_blk.calData[ CHANNEL_PHASE_B_VOLTAGE ] );
    s_ctl_blk.siData.channel[ CHANNEL_PHASE_C_VOLTAGE ] = compute_phase_voltage( s_ctl_blk.calData[ CHANNEL_PHASE_C_VOLTAGE ] );


    // if( abs( s_ctl_blk.siData.channel[ CHANNEL_PHASE_A_CURRENT ] ) < 0.02f )
    // {
    //   s_ctl_blk.siData.channel[ CHANNEL_PHASE_A_CURRENT ] = 0.0f;
    // }

    // if( abs( s_ctl_blk.siData.channel[ CHANNEL_PHASE_B_CURRENT ] ) < 0.02f )
    // {
    //   s_ctl_blk.siData.channel[ CHANNEL_PHASE_B_CURRENT ] = 0.0f;
    // }

    // if( abs( s_ctl_blk.siData.channel[ CHANNEL_PHASE_C_CURRENT ] ) < 0.02f )
    // {
    //   s_ctl_blk.siData.channel[ CHANNEL_PHASE_C_CURRENT ] = 0.0f;
    // }

    /*-------------------------------------------------------------------------
    Invoke the user callback
    -------------------------------------------------------------------------*/
    if ( s_ctl_blk.callback )
    {
      s_ctl_blk.callback();
    }
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  Chimera::Timer::Trigger::Slave* getTimer()
  {
    return &s_motor_sense_timer;
  }


  void initialize()
  {
    /*-------------------------------------------------------------------------
    Reset the state of the module
    -------------------------------------------------------------------------*/
    s_ctl_blk.siData.timestamp = current_time_sec();

    for ( auto i = 0; i < CHANNEL_COUNT; i++ )
    {
      s_ctl_blk.calData[ i ]        = 0.0f;
      s_ctl_blk.calOffset[ i ]      = 0.0f;
      s_ctl_blk.rawData[ i ]        = 0.0f;
      s_ctl_blk.siData.channel[ i ] = 0.0f;
    }

    s_ctl_blk.dbg_pin = Chimera::GPIO::getDriver( Orbit::IO::Digital::dbg1Port, Orbit::IO::Digital::dbg1Pin );
    s_ctl_blk.dbg_pin->setState( Chimera::GPIO::State::LOW );

    /*-------------------------------------------------------------------------
    Calibrate the sense inputs
    -------------------------------------------------------------------------*/
    calibrate();

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this module's ISR handler.
    This ADC should already be pre-configured to listen for timer events.
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<isr_on_motor_sense_adc_conversion_complete>();

    auto pADC = Chimera::ADC::getDriver( Orbit::IO::Analog::MotorADC );
    pADC->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    /*-------------------------------------------------------------------------
    Configure Timer 8 to trigger ADC conversions at a fixed rate
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::SlaveConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.coreConfig.instance    = Orbit::IO::Timer::MotorSense;
    trig_cfg.coreConfig.baseFreq    = TIMER_BASE_FREQ;
    trig_cfg.coreConfig.tolerance   = 1.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;
    trig_cfg.frequency              = Orbit::Data::SysControl.statorPWMFreq;
    trig_cfg.trigSyncAction         = Chimera::Timer::Trigger::SyncAction::SYNC_RESET;
    trig_cfg.trigSyncSignal         = Chimera::Timer::Trigger::Signal::TRIG_SIG_0; /**< ITR0: TIM1 TRGO->TIM8*/

    RT_HARD_ASSERT( Chimera::Status::OK == s_motor_sense_timer.init( trig_cfg ) );

    /*-------------------------------------------------------------------------
    From running the SampleTimeOptimizer routine, I was able to experimentally
    determine that setting this offset to 45 will result in the ADC sampling
    with enough overhead, down to about 6% duty cycle. We'll see if this is ok.
    -------------------------------------------------------------------------*/
    s_motor_sense_timer.setEventOffset( 0 );
    s_motor_sense_timer.enable();

    /*-------------------------------------------------------------------------
    Enable the ADC peripheral after the timer initializes to prevent any
    spurrious triggering.
    -------------------------------------------------------------------------*/
    pADC->startSequence();
  }


  void reset()
  {
    // TODO: Don't call this unless you have a way to restart the power up sequence.
    // auto pADC = Chimera::ADC::getDriver( Orbit::IO::Analog::MotorADC );
    // RT_HARD_ASSERT( pADC );
    // pADC->close();
  }


  void onComplete( SenseCallback callback )
  {
    s_ctl_blk.callback = callback;
  }


  volatile const SenseData &getSenseData()
  {
    return s_ctl_blk.siData;
  }


  void calibrate()
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    const Chimera::ADC::Channel sample_channels[] = {
      IO::Analog::adcIPhaseA, /**< CHANNEL_PHASE_A_CURRENT */
      IO::Analog::adcIPhaseB, /**< CHANNEL_PHASE_B_CURRENT */
      IO::Analog::adcIPhaseC, /**< CHANNEL_PHASE_C_CURRENT */
      IO::Analog::adcVPhaseA, /**< CHANNEL_PHASE_A_VOLTAGE */
      IO::Analog::adcVPhaseB, /**< CHANNEL_PHASE_B_VOLTAGE */
      IO::Analog::adcVPhaseC  /**< CHANNEL_PHASE_C_VOLTAGE */
    };

    static_assert( CHANNEL_PHASE_A_CURRENT == 0 );
    static_assert( CHANNEL_PHASE_B_CURRENT == 1 );
    static_assert( CHANNEL_PHASE_C_CURRENT == 2 );
    static_assert( CHANNEL_PHASE_A_VOLTAGE == 3 );
    static_assert( CHANNEL_PHASE_B_VOLTAGE == 4 );
    static_assert( CHANNEL_PHASE_C_VOLTAGE == 5 );
    static_assert( ARRAY_COUNT( sample_channels ) == CHANNEL_COUNT );

    /*-------------------------------------------------------------------------
    Relinquish control from the timer peripheral of the motor power stage.
    -------------------------------------------------------------------------*/
    TIMER::configureIOTesting();

    /*-------------------------------------------------------------------------
    Short the low side of the motor drive to ground. This assumes the motor has
    been connected, which creates the ground path.
    -------------------------------------------------------------------------*/
    auto pin1 = Chimera::GPIO::getDriver( IO::Timer::portT1Ch1N, IO::Timer::pinT1Ch1N );
    auto pin2 = Chimera::GPIO::getDriver( IO::Timer::portT1Ch2N, IO::Timer::pinT1Ch2N );
    auto pin3 = Chimera::GPIO::getDriver( IO::Timer::portT1Ch3N, IO::Timer::pinT1Ch3N );

    pin1->setState( Chimera::GPIO::State::HIGH );
    pin2->setState( Chimera::GPIO::State::HIGH );
    pin3->setState( Chimera::GPIO::State::HIGH );

    /* Allow time for settling */
    Chimera::delayMilliseconds( 5 );

    /*-------------------------------------------------------------------------
    Measure the DC offset of the motor phase current/voltage sensors
    -------------------------------------------------------------------------*/
    auto adc = Chimera::ADC::getDriver( IO::Analog::MotorADC );

    for ( size_t idx = 0; idx < ARRAY_COUNT( sample_channels ); idx++ )
    {
      float  ceiling   = -FLT_MAX;
      float  floor     = FLT_MAX;
      float  samples   = 0.0f;
      float  pIxAvg    = 0.0f;
      size_t startTime = Chimera::millis();

      while ( ( Chimera::millis() - startTime ) < Chimera::Thread::TIMEOUT_5MS )
      {
        auto  sample  = adc->sampleChannel( sample_channels[ idx ] );
        float voltage = adc->toVoltage( sample );
        pIxAvg += voltage;
        samples++;

        if ( voltage > ceiling )
        {
          ceiling = voltage;
        }
        else if ( voltage < floor )
        {
          floor = voltage;
        }
      }

      s_ctl_blk.calOffset[ idx ] = ( pIxAvg / samples );
    }

    /*-------------------------------------------------------------------------
    Give control back to the timer peripheral
    -------------------------------------------------------------------------*/
    TIMER::configureIOControl();
  }

}    // namespace Orbit::Motor
