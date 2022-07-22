/******************************************************************************
 *  File Name:
 *    orbit_adc.cpp
 *
 *  Description:
 *    Orbit ESC ADC driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/hw/orbit_adc.hpp>
#include <src/config/bsp/board_map.hpp>


namespace Orbit::ADC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::ADC::ChannelList s_adc_channels;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    Chimera::GPIO::Driver_rPtr gpio;
    Chimera::GPIO::PinInit     pin_cfg;

    /*-------------------------------------------------------------------------
    Phase A Current GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portPhaseA;
    pin_cfg.pin      = IO::Analog::pinPhaseA;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase B Current GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portPhaseB;
    pin_cfg.pin      = IO::Analog::pinPhaseB;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Power Supply Voltage GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portVSupply;
    pin_cfg.pin      = IO::Analog::pinVSupply;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

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
    adc_cfg.clockPrescale       = Chimera::ADC::Prescaler::DIV_2;
    adc_cfg.clockSource         = Chimera::Clock::Bus::SYSCLK;
    adc_cfg.oversampleRate      = Chimera::ADC::Oversampler::OS_NONE;
    adc_cfg.periph              = IO::Analog::peripheral;
    adc_cfg.resolution          = Chimera::ADC::Resolution::BIT_12;
    adc_cfg.transferMode        = Chimera::ADC::TransferMode::DMA;

    adc = Chimera::ADC::getDriver( adc_cfg.periph );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->open( adc_cfg ) );

    /* Configure the sequence conversion */
    s_adc_channels[ 0 ] = IO::Analog::adcPhaseA;
    s_adc_channels[ 1 ] = IO::Analog::adcPhaseB;
    s_adc_channels[ 2 ] = IO::Analog::adcVSupply;

    seq.clear();
    seq.channels    = &s_adc_channels;
    seq.numChannels = 3;
    seq.seqMode     = Chimera::ADC::SamplingMode::TRIGGER;
    seq.trigMode    = Chimera::ADC::TriggerMode::RISING_EDGE;
    seq.trigChannel = 10; // Regular channel, TIM1_TRGO2

    RT_HARD_ASSERT( Chimera::Status::OK == adc->configSequence( seq ) );

    /*-------------------------------------------------------------------------
    Kick off the ADC sequence sampling
    -------------------------------------------------------------------------*/
    adc->startSequence();
  }

}    // namespace Orbit::ADC
