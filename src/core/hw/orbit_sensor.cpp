/******************************************************************************
 *  File Name:
 *    orbit_sensor.cpp
 *
 *  Description:
 *    Implementation of sensor interface for the OrbitESC board
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/hw/orbit_sensor.hpp>
#include <src/config/bsp/board_map.hpp>
#include <Chimera/adc>

namespace Orbit::Sensor
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::ADC::Driver_rPtr       s_adc_driver;
  static Chimera::Timer::Trigger::Master s_sensor_timer;

  static volatile uint16_t s_adc_samples[ 4 ];

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Callback to handle results of ADC conversions
   *
   * @param isr  Results of the ADC conversion
   */
  static void adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr )
  {
    for( auto i = 0; i < isr.num_samples; i++ )
    {
      s_adc_samples[ 0 ] = isr.samples[ i ];
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    s_adc_driver = Chimera::ADC::getDriver( IO::Analog::SensorPeripheral );
    RT_DBG_ASSERT( s_adc_driver );

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this module's ISR handler
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<adcISRTxfrComplete>();
    s_adc_driver->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    /*-------------------------------------------------------------------------
    Configure the timer for triggering ADC samples
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = 10;
    trig_cfg.coreConfig.instance    = Orbit::IO::Timer::SensorCapture;
    trig_cfg.coreConfig.baseFreq    = 10'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == s_sensor_timer.init( trig_cfg ) );
    s_sensor_timer.enable();

    /*-------------------------------------------------------------------------
    Enable the ADC to start sampling
    -------------------------------------------------------------------------*/
    s_adc_driver->startSequence();
  }


  float getTemperatureCelcius()
  {
    /*-------------------------------------------------------------------------
    Resistor Divider
    -------------------------------------------------------------------------*/
    constexpr float NTC_NOM_RES  = 10000.0f;    // NTC
    constexpr float NTC_BETA     = 3413.0f;     // Beta value @25-75C
    constexpr float NTC_NOM_TEMP = 25.0f;       // Nominal temperature
    constexpr float R2           = 10000.0f;

    /*-------------------------------------------------------------------------
    Get the ADC reading
    -------------------------------------------------------------------------*/
    auto sample  = s_adc_driver->sampleChannel( IO::Analog::adcVTemp );
    auto voltage = s_adc_driver->toVoltage( sample );

    /*-------------------------------------------------------------------------
    Calculate the temperature in Celsius using the Steinhart-Hart equation
    -------------------------------------------------------------------------*/
    float Rntc      = 0.0f;
    float steinhart = 0.0f;

    Rntc      = R2 * ( voltage / ( s_adc_driver->analogReference() - voltage ) );
    steinhart = Rntc / NTC_NOM_RES;                    // (R/R0)
    steinhart = log( steinhart );                      // ln(R/R0)
    steinhart /= NTC_BETA;                             // 1/B * ln(R/R0)
    steinhart += 1.0f / ( NTC_NOM_TEMP + 273.15f );    // + (1/T0)
    steinhart = 1.0f / steinhart;                      // Invert
    steinhart -= 273.15f;                              // Convert to Celsius

    return steinhart;
  }


  float getSupplyVoltage()
  {
    /*-------------------------------------------------------------------------
    Resistor Divider
    -------------------------------------------------------------------------*/
    constexpr float R1 = 10000.0f;
    constexpr float R2 = 1500.0f;

    /*-------------------------------------------------------------------------
    Get the ADC reading
    -------------------------------------------------------------------------*/
    auto sample  = s_adc_driver->sampleChannel( IO::Analog::adcVSupply );
    auto voltage = s_adc_driver->toVoltage( sample );

    return ( voltage * ( R1 + R2 ) ) / R2;
  }


  float getMCUVoltage()
  {
    /*-------------------------------------------------------------------------
    Resistor Divider
    -------------------------------------------------------------------------*/
    constexpr float R1 = 10000.0f;
    constexpr float R2 = 1500.0f;

    /*-------------------------------------------------------------------------
    Get the ADC reading
    -------------------------------------------------------------------------*/
    auto sample  = s_adc_driver->sampleChannel( IO::Analog::adcVMCU );
    auto voltage = s_adc_driver->toVoltage( sample );

    return ( voltage * ( R1 + R2 ) ) / R2;
  }


  float getCurrentSenseReferenceVoltage()
  {
    auto sample  = s_adc_driver->sampleChannel( IO::Analog::adcVISenseRef );
    auto voltage = s_adc_driver->toVoltage( sample );
    return voltage;
  }

}    // namespace Orbit::Sensor
