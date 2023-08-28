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
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_timer.hpp>


namespace Orbit::ADC
{
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
    Temperature Instrumentation Voltage GPIO Config
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
    adc_cfg.bmISREnable     = Chimera::ADC::Interrupt::EOC_SEQUENCE;
    adc_cfg.clockPrescale   = Chimera::ADC::PreScaler::DIV_2;
    adc_cfg.clockSource     = Chimera::Clock::Bus::SYSCLK;
    adc_cfg.overSampleRate  = Chimera::ADC::OverSampler::OS_NONE;
    adc_cfg.overSampleShift = Chimera::ADC::OverSampleShift::OS_NONE;
    adc_cfg.periph          = IO::Analog::MotorADC;
    adc_cfg.resolution      = Chimera::ADC::Resolution::BIT_12;
    adc_cfg.transferMode    = Chimera::ADC::TransferMode::DMA;
    adc_cfg.analogVRef      = 3.30f;

    adc = Chimera::ADC::getDriver( adc_cfg.periph );
    RT_DBG_ASSERT( adc );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->open( adc_cfg ) );

    /*-------------------------------------------------------------------------
    Sequence conversion configuration
    -------------------------------------------------------------------------*/
    s_motor_channels.fill( Chimera::ADC::Channel::UNKNOWN );
    s_motor_channels[ Motor::CHANNEL_PHASE_A_CURRENT ] = IO::Analog::adcIPhaseA;
    s_motor_channels[ Motor::CHANNEL_PHASE_B_CURRENT ] = IO::Analog::adcIPhaseB;
    s_motor_channels[ Motor::CHANNEL_PHASE_C_CURRENT ] = IO::Analog::adcIPhaseC;
    s_motor_channels[ Motor::CHANNEL_PHASE_A_VOLTAGE ] = IO::Analog::adcVPhaseA;
    s_motor_channels[ Motor::CHANNEL_PHASE_B_VOLTAGE ] = IO::Analog::adcVPhaseB;
    s_motor_channels[ Motor::CHANNEL_PHASE_C_VOLTAGE ] = IO::Analog::adcVPhaseC;

    seq.clear();
    seq.channels    = &s_motor_channels;
    seq.numChannels = Motor::CHANNEL_COUNT;
    seq.seqGroup    = Chimera::ADC::SequenceGroup::REGULAR;
    seq.seqMode     = Chimera::ADC::SamplingMode::TRIGGER;
    seq.trigMode    = Chimera::ADC::TriggerMode::RISING_EDGE;
    seq.trigChannel = IO::Analog::MotorExternalEventChannel;

    RT_HARD_ASSERT( Chimera::Status::OK == adc->configSequence( seq ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcIPhaseA, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVPhaseA, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcIPhaseB, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVPhaseB, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcIPhaseC, 12 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVPhaseC, 12 ) );
  }


  /**
   * @brief Configures the ADC channels used for the instrumentation inputs
   */
  static void cfg_instrumentation_adc()
  {
    Chimera::ADC::Driver_rPtr  adc;
    Chimera::ADC::DriverConfig adc_cfg;
    Chimera::ADC::SequenceInit seq;

    /*-------------------------------------------------------------------------
    Core peripheral configuration
    -------------------------------------------------------------------------*/
    adc_cfg.clear();
    adc_cfg.periph          = IO::Analog::InstrADC;
    adc_cfg.bmISREnable     = Chimera::ADC::Interrupt::EOC_SEQUENCE;
    adc_cfg.clockPrescale   = Chimera::ADC::PreScaler::DIV_2;
    adc_cfg.clockSource     = Chimera::Clock::Bus::SYSCLK;
    adc_cfg.overSampleRate  = Chimera::ADC::OverSampler::OS_NONE;
    adc_cfg.overSampleShift = Chimera::ADC::OverSampleShift::OS_NONE;
    adc_cfg.resolution      = Chimera::ADC::Resolution::BIT_12;
    adc_cfg.transferMode    = Chimera::ADC::TransferMode::DMA;
    adc_cfg.analogVRef      = 3.30f;

    adc = Chimera::ADC::getDriver( adc_cfg.periph );
    RT_DBG_ASSERT( adc );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->open( adc_cfg ) );

    /*-------------------------------------------------------------------------
    Configure the measurement sequence to be triggered. Expects a system timer
    to be initialized elsewhere to trigger the ADC at a fixed rate.
    -------------------------------------------------------------------------*/
    s_sensor_channels.fill( Chimera::ADC::Channel::UNKNOWN );
    s_sensor_channels[ Instrumentation::CHANNEL_VSUPPLY ] = IO::Analog::adcVSupply;
    s_sensor_channels[ Instrumentation::CHANNEL_VMCU ]    = IO::Analog::adcVMCU;
    s_sensor_channels[ Instrumentation::CHANNEL_TEMP ]    = IO::Analog::adcVTemp;
    s_sensor_channels[ Instrumentation::CHANNEL_VREF ]    = IO::Analog::adcVISenseRef;

    seq.clear();
    seq.channels    = &s_sensor_channels;
    seq.numChannels = Instrumentation::CHANNEL_COUNT;
    seq.seqGroup    = Chimera::ADC::SequenceGroup::REGULAR;
    seq.seqMode     = Chimera::ADC::SamplingMode::TRIGGER;
    seq.trigMode    = Chimera::ADC::TriggerMode::RISING_EDGE;
    seq.trigChannel = IO::Analog::SensorExternalEventChannel;

    RT_HARD_ASSERT( Chimera::Status::OK == adc->configSequence( seq ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVSupply, 80 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVMCU, 80 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVTemp, 80 ) );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->setSampleTime( IO::Analog::adcVISenseRef, 80 ) );
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    init_gpio_pins();
    cfg_motor_adc();
    cfg_instrumentation_adc();
  }


  uint32_t motorChannelSampleTimeNs()
  {
    using namespace Chimera::Timer;
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Compute the total time it takes to sample all the motor channels in
    nanoseconds. This is used to configure the ADC trigger timing.
    -------------------------------------------------------------------------*/
    uint32_t sampleTimeNs = 0;
    auto     adc          = Chimera::ADC::getDriver( IO::Analog::MotorADC );

    for ( size_t idx = 0; idx < Motor::CHANNEL_COUNT; idx++ )
    {
      sampleTimeNs += adc->totalMeasureTime( adc->getSampleCycle( s_motor_channels[ idx ] ) );
    }

    return sampleTimeNs;
  }

}    // namespace Orbit::ADC
