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
  static Chimera::ADC::SampleList s_adc_samples;
  static Chimera::ADC::ChannelList s_adc_channels;


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void adc_isr_handler( const Chimera::ADC::InterruptDetail &detail );


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    Chimera::GPIO::Driver_rPtr gpio;
    Chimera::GPIO::PinInit pin_cfg;

    /*-------------------------------------------------------------------------
    Phase A GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portPhaseA;
    pin_cfg.pin      = IO::Analog::pinPhaseA;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase B GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portPhaseB;
    pin_cfg.pin      = IO::Analog::pinPhaseB;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Phase C GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portPhaseC;
    pin_cfg.pin      = IO::Analog::pinPhaseC;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Three Phase Center Tap GPIO Config
    -------------------------------------------------------------------------*/
    pin_cfg          = IO::Analog::CommonAnalogCfg;
    pin_cfg.port     = IO::Analog::portCenterTap;
    pin_cfg.pin      = IO::Analog::pinCenterTap;
    pin_cfg.validity = true;

    gpio = Chimera::GPIO::getDriver( pin_cfg.port, pin_cfg.pin );
    RT_HARD_ASSERT( Chimera::Status::OK == gpio->init( pin_cfg ) );

    /*-------------------------------------------------------------------------
    Configure the ADC driver
    -------------------------------------------------------------------------*/
    Chimera::ADC::Driver_rPtr adc;
    Chimera::ADC::DriverConfig adc_cfg;
    Chimera::ADC::SequenceInit seq;

    /* Core configuration */
    adc_cfg.clear();
    adc_cfg.bmISREnable         = Chimera::ADC::Interrupt::EOC_SEQUENCE | Chimera::ADC::Interrupt::EOC_SINGLE | Chimera::ADC::Interrupt::OVERRUN;
    adc_cfg.clockPrescale       = Chimera::ADC::Prescaler::DIV_2;
    adc_cfg.clockSource         = Chimera::Clock::Bus::SYSCLK;
    adc_cfg.defaultSampleCycles = 24;
    adc_cfg.oversampleRate      = Chimera::ADC::Oversampler::OS_NONE;
    adc_cfg.periph              = Chimera::ADC::Peripheral::ADC_1;
    adc_cfg.resolution          = Chimera::ADC::Resolution::BIT_12;
    adc_cfg.transferMode        = Chimera::ADC::TransferMode::INTERRUPT;

    adc = Chimera::ADC::getDriver( adc_cfg.periph );
    RT_HARD_ASSERT( Chimera::Status::OK == adc->open( adc_cfg ) );

    /* Attach project side interrupt handler */
    adc->onInterrupt( adc_cfg.bmISREnable, Chimera::ADC::ISRCallback::create<adc_isr_handler>() );

    /* Configure the sequence conversion */
    s_adc_channels[ 0 ] = IO::Analog::adcPhaseA;
    s_adc_channels[ 1 ] = IO::Analog::adcPhaseB;
    s_adc_channels[ 2 ] = IO::Analog::adcPhaseC;
    s_adc_channels[ 3 ] = IO::Analog::adcCenterTap;

    seq.clear();
    seq.channels    = &s_adc_channels;
    seq.numChannels = 4;
    seq.mode        = Chimera::ADC::SamplingMode::ONE_SHOT;

    RT_HARD_ASSERT( Chimera::Status::OK == adc->configSequence( seq ) );
  }

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Processes sampled ADC data
   *
   * This function is registered as a callback with the ADC driver's userspace
   * interrupt handler, allowing normal multi-tasking operations to be used
   * safely.
   *
   * @param detail    Details about the interrupt event
   */
  static void adc_isr_handler( const Chimera::ADC::InterruptDetail &detail )
  {
  }
}  // namespace Orbit::ADC
