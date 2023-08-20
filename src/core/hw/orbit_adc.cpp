/******************************************************************************
 *  File Name:
 *    orbit_adc.cpp
 *
 *  Description:
 *    OrbitESC ADC driver
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <etl/delegate.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_timer.hpp>


namespace Orbit::ADC
{
  /*---------------------------------------------------------------------------
  Voltage Sense Constants
  ---------------------------------------------------------------------------*/
  static constexpr float VDC_SENSE_R1          = 10'000.0f;
  static constexpr float VDC_SENSE_R2          = 1'500.0f;
  static constexpr float VDC_DIV_RATIO         = VDC_SENSE_R2 / ( VDC_SENSE_R1 + VDC_SENSE_R2 );
  static constexpr float ADC_TO_VDC_CONV_CONST = 1.0f / VDC_DIV_RATIO;

  /*---------------------------------------------------------------------------
  Current Sense Constants
  ---------------------------------------------------------------------------*/
  static constexpr float ISHUNT_AMP_GAIN = 10.0f;    // Configured gain of all amplifiers
  static constexpr float ISENSE_VREF     = 1.65f;    // Voltage reference for current sense
  static constexpr float RSHUNT_OHM      = 0.01f;    // Low-side shunt resistor

  /*---------------------------------------------------------------------------
  Current Limit Constants
  ---------------------------------------------------------------------------*/
  static constexpr float ADC_WDG_I_LIMIT = 2.0f;

  /*---------------------------------------------------------------------------
  Voltage Limit Constants
  ---------------------------------------------------------------------------*/
  static constexpr float ADC_WDG_V_HI_LIMIT = 15.0f;
  static constexpr float ADC_WDG_V_LO_LIMIT = 2.0f;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::ADC::ChannelList s_motor_channels;
  static Chimera::ADC::ChannelList s_sensor_channels;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void adc_watchdog_monitor_isr( const Chimera::ADC::InterruptDetail &isrData )
  {
    // using namespace Orbit::Control;
    // const auto mode = FOCDriver.currentMode();
    // if ( ( mode != ModeId::IDLE ) && ( mode != ModeId::FAULT ) )
    // {
    //   FOCDriver.sendSystemEvent( EventId::EMERGENCY_HALT );
    // }
  }

  /**
   * @brief Initializes the GPIO pins used for ADC input.
   */
  static void init_gpio_pins()
  {
    Chimera::GPIO::Driver_rPtr gpio;
    Chimera::GPIO::PinInit     pin_cfg;

    /*-------------------------------------------------------------------------
    Phase A Current GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portIPhaseA;
    pin_cfg.pin      = IO::Analog::pinIPhaseA;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase A Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVPhaseA;
    pin_cfg.pin      = IO::Analog::pinVPhaseA;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase B Current GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portIPhaseB;
    pin_cfg.pin      = IO::Analog::pinIPhaseB;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase B Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVPhaseB;
    pin_cfg.pin      = IO::Analog::pinVPhaseB;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase C Current GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portIPhaseC;
    pin_cfg.pin      = IO::Analog::pinIPhaseC;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase C Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVPhaseC;
    pin_cfg.pin      = IO::Analog::pinVPhaseC;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    System Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVSupply;
    pin_cfg.pin      = IO::Analog::pinVSupply;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    MCU Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVMCU;
    pin_cfg.pin      = IO::Analog::pinVMCU;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Temperature Sensor Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVTemp;
    pin_cfg.pin      = IO::Analog::pinVTemp;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Current Sense Reference Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVISenseRef;
    pin_cfg.pin      = IO::Analog::pinVISenseRef;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_DBG_ASSERT( gpio );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );
  }


  /**
   * @brief Configures the ADC driver for the motor control system
   */
  static void cfg_motor_adc()
  {
    Chimera::ADC::Driver_rPtr  adc;
    Chimera::ADC::DriverConfig adc_cfg;
    Chimera::ADC::SequenceInit seq;

    /*-------------------------------------------------------------------------
    Core configuration
    -------------------------------------------------------------------------*/
    adc_cfg.clear();
    adc_cfg.defaultSampleCycles = 1000;
    adc_cfg.bmISREnable         = Chimera::ADC::Interrupt::EOC_SEQUENCE;
    adc_cfg.clockPrescale       = Chimera::ADC::PreScaler::DIV_2;
    adc_cfg.clockSource         = Chimera::Clock::Bus::SYSCLK;
    adc_cfg.overSampleRate      = Chimera::ADC::OverSampler::OS_NONE;
    adc_cfg.overSampleShift     = Chimera::ADC::OverSampleShift::OS_NONE;
    adc_cfg.periph              = IO::Analog::MotorPeripheral;
    adc_cfg.resolution          = Chimera::ADC::Resolution::BIT_12;
    adc_cfg.transferMode        = Chimera::ADC::TransferMode::DMA;

    adc = Chimera::ADC::getDriver( adc_cfg.periph );
    RT_DBG_ASSERT( adc );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->open( adc_cfg ) );

    /*-------------------------------------------------------------------------
    Sequence conversion configuration
    -------------------------------------------------------------------------*/
    s_motor_channels.fill( Chimera::ADC::Channel::UNKNOWN );
    s_motor_channels[ 0 ] = IO::Analog::adcVPhaseA;
    s_motor_channels[ 1 ] = IO::Analog::adcVPhaseB;
    s_motor_channels[ 2 ] = IO::Analog::adcVPhaseC;
    s_motor_channels[ 3 ] = IO::Analog::adcIPhaseA;
    s_motor_channels[ 4 ] = IO::Analog::adcIPhaseB;
    s_motor_channels[ 5 ] = IO::Analog::adcIPhaseC;

    seq.clear();
    seq.channels    = &s_motor_channels;
    seq.numChannels = 6;
    seq.seqGroup    = Chimera::ADC::SequenceGroup::REGULAR;
    seq.seqMode     = Chimera::ADC::SamplingMode::TRIGGER;
    seq.trigMode    = Chimera::ADC::TriggerMode::RISING_EDGE;
    seq.trigChannel = 0;  // EXTSEL Regular Channel, TIM1_CC1

    RT_HARD_ASSERT( Chimera::Status::OK == adc->configSequence( seq ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcIPhaseA, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVPhaseA, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcIPhaseB, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVPhaseB, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcIPhaseC, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVPhaseC, 12 ) );
  }


  /**
   * @brief Configures the ADC channels used for the sensor inputs
   */
  static void cfg_sensor_adc()
  {
    /*-------------------------------------------------------------------------
    Configure the ADC driver
    -------------------------------------------------------------------------*/
    Chimera::ADC::Driver_rPtr  adc;
    Chimera::ADC::DriverConfig adc_cfg;
    Chimera::ADC::SequenceInit seq;

    /* Core configuration */
    adc_cfg.clear();
    adc_cfg.defaultSampleCycles = 1000;
    adc_cfg.bmISREnable         = Chimera::ADC::Interrupt::EOC_SEQUENCE;
    adc_cfg.clockPrescale       = Chimera::ADC::PreScaler::DIV_2;
    adc_cfg.clockSource         = Chimera::Clock::Bus::SYSCLK;
    adc_cfg.overSampleRate      = Chimera::ADC::OverSampler::OS_NONE;
    adc_cfg.overSampleShift     = Chimera::ADC::OverSampleShift::OS_NONE;
    adc_cfg.periph              = IO::Analog::SensorPeripheral;
    adc_cfg.resolution          = Chimera::ADC::Resolution::BIT_12;
    adc_cfg.transferMode        = Chimera::ADC::TransferMode::DMA;

    adc = Chimera::ADC::getDriver( adc_cfg.periph );
    RT_DBG_ASSERT( adc );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->open( adc_cfg ) );

    /* Configure the sequence conversion */
    s_sensor_channels.fill( Chimera::ADC::Channel::UNKNOWN );
    s_sensor_channels[ 0 ] = IO::Analog::adcVSupply;
    s_sensor_channels[ 1 ] = IO::Analog::adcVMCU;
    s_sensor_channels[ 2 ] = IO::Analog::adcVTemp;
    s_sensor_channels[ 3 ] = IO::Analog::adcVISenseRef;

    seq.clear();
    seq.channels    = &s_sensor_channels;
    seq.numChannels = 4;
    seq.seqGroup    = Chimera::ADC::SequenceGroup::REGULAR;
    seq.seqMode     = Chimera::ADC::SamplingMode::TRIGGER;
    seq.trigMode    = Chimera::ADC::TriggerMode::RISING_EDGE;
    seq.trigChannel = 8;    // EXTSEL Regular channel, TIM3_TRGO, OC1REF

    RT_HARD_ASSERT( Chimera::Status::OK == adc->configSequence( seq ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVSupply, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVMCU, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVTemp, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVISenseRef, 12 ) );
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    init_gpio_pins();
    cfg_motor_adc();
    cfg_sensor_adc();
  }


  void startSampling()
  {
    auto adc_motor = Chimera::ADC::getDriver( IO::Analog::MotorPeripheral );
    adc_motor->startSequence();

    auto adc_sensor = Chimera::ADC::getDriver( IO::Analog::SensorPeripheral );
    adc_sensor->startSequence();
  }


  void stopSampling()
  {
    auto adc_motor = Chimera::ADC::getDriver( IO::Analog::MotorPeripheral );
    adc_motor->stopSequence();

    auto adc_sensor = Chimera::ADC::getDriver( IO::Analog::SensorPeripheral );
    adc_sensor->stopSequence();
  }


  void calibrateCurrentSensors( IPhaseCalArray &cal, const size_t sampleTimeMs )
  {
    const Chimera::ADC::Channel sample_channels[] = { IO::Analog::adcIPhaseA, IO::Analog::adcIPhaseB, IO::Analog::adcIPhaseC };

    /*-------------------------------------------------------------------------
    Ensure the motor is off for calibration
    -------------------------------------------------------------------------*/
    // TODO: Add a check to ensure the motor is off

    /*-------------------------------------------------------------------------
    Short the low side of the motor drive to ground. This assumes the motor has
    been connected, which creates the ground path.
    -------------------------------------------------------------------------*/
    TIMER::configureIOTesting();
    auto pin1 = Chimera::GPIO::getDriver( IO::Timer::portT1Ch1N, IO::Timer::pinT1Ch1N );
    auto pin2 = Chimera::GPIO::getDriver( IO::Timer::portT1Ch2N, IO::Timer::pinT1Ch2N );
    auto pin3 = Chimera::GPIO::getDriver( IO::Timer::portT1Ch3N, IO::Timer::pinT1Ch3N );

    pin1->setState( Chimera::GPIO::State::HIGH );
    pin2->setState( Chimera::GPIO::State::HIGH );
    pin3->setState( Chimera::GPIO::State::HIGH );

    Chimera::delayMilliseconds( 100 );

    /*-------------------------------------------------------------------------
    Measure the DC offset of the motor phase current sensors
    -------------------------------------------------------------------------*/
    LOG_TRACE( "Calibrating phase current sensors\r\n" );
    auto adc = Chimera::ADC::getDriver( IO::Analog::MotorPeripheral );

    for( int idx = 0; idx <= 3; idx++ )
    {
      cal[ idx ].ceiling  = -FLT_MAX;
      cal[ idx ].floor    = FLT_MAX;
      cal[ idx ].dcOffset = 0.0f;

      float samples = 0.0f;
      float pIxAvg  = 0.0f;
      size_t startTime = Chimera::millis();

      while ( ( Chimera::millis() - startTime ) < sampleTimeMs )
      {
        auto sample = adc->sampleChannel( sample_channels[ idx ] );
        float voltage = adc->toVoltage( sample );
        pIxAvg += voltage;
        samples++;

        if( voltage > cal[idx].ceiling )
        {
          cal[idx].ceiling = voltage;
        }
        else if( voltage < cal[idx].floor )
        {
          cal[idx].floor = voltage;
        }
      }

      cal[ idx ].dcOffset = ( pIxAvg / samples );
    }

    LOG_TRACE( "Done\r\n" );
  }


  float sample2PhaseCurrent( const float vin )
  {
    /*-------------------------------------------------------------------------
    Calculate backwards from the ADC reading to what the real current is. The
    op-amp is configured as an inverting amplifier, so a positive current will
    result in a negative voltage delta. The current is then calculated by
    dividing the voltage by the shunt resistor and the op-amp gain.
    -------------------------------------------------------------------------*/
    float raw = -1.0f * ( vin / ( ISHUNT_AMP_GAIN * RSHUNT_OHM ) );
    return raw;
  }


  float sample2BusVoltage( const float vin )
  {
    return vin * ADC_TO_VDC_CONV_CONST;
  }

}    // namespace Orbit::ADC
