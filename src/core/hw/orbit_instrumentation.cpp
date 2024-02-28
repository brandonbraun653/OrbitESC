/******************************************************************************
 *  File Name:
 *    orbit_instrumentation.cpp
 *
 *  Description:
 *    Implementation of sensor interface for the OrbitESC board
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/config/bsp/board_map.hpp>
#include <Chimera/adc>

namespace Orbit::Instrumentation
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float SAMPLE_RATE_HZ = 10.0f;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Trigger::Master s_sensor_timer;
  static volatile float                  s_adc_vref;
  static volatile float                  s_adc_vres;
  static volatile uint16_t               s_adc_samples[ CHANNEL_COUNT ];

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
    return ( static_cast<float>( counts ) * s_adc_vref ) / s_adc_vres;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    auto pADC = Chimera::ADC::getDriver( IO::Analog::InstrADC );
    RT_DBG_ASSERT( pADC );

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this module's ISR handler
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback =
        Chimera::ADC::ISRCallback::create<Private::isr_on_instrumentation_adc_conversion_complete>();
    pADC->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    /*-------------------------------------------------------------------------
    Configure the timer for triggering ADC samples
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = SAMPLE_RATE_HZ;
    trig_cfg.coreConfig.instance    = Orbit::IO::Timer::SensorCapture;
    trig_cfg.coreConfig.baseFreq    = 10'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == s_sensor_timer.init( trig_cfg ) );
    s_sensor_timer.enable();

    /*-------------------------------------------------------------------------
    Enable the ADC to start sampling
    -------------------------------------------------------------------------*/
    pADC->startSequence();
  }


  float getTemperatureCelcius()
  {
    /*-------------------------------------------------------------------------
    Resistor Divider
    -------------------------------------------------------------------------*/
    constexpr float NTC_NOM_RES  = 10000.0f;    // NTC
    constexpr float NTC_BETA     = 3413.0f;     // Beta value @25-75C
    constexpr float NTC_NOM_TEMP = 25.0f;       // Nominal temperature
    constexpr float R2           = 10000.0f;    // Resistor divider

    /*-------------------------------------------------------------------------
    Get the ADC reading
    -------------------------------------------------------------------------*/
    const float voltage = counts_to_voltage( s_adc_samples[ CHANNEL_TEMP ] );

    /*-------------------------------------------------------------------------
    Calculate the temperature in Celsius using the Steinhart-Hart equation
    -------------------------------------------------------------------------*/
    float Rntc      = 0.0f;
    float steinhart = 0.0f;

    Rntc      = R2 * ( voltage / ( s_adc_vref - voltage ) );
    steinhart = Rntc / NTC_NOM_RES;                    // (R/R0)
    steinhart = log( steinhart );                      // ln(R/R0)
    steinhart /= NTC_BETA;                             // 1/B * ln(R/R0)
    steinhart += 1.0f / ( NTC_NOM_TEMP + 273.15f );    // + (1/T0)
    steinhart = 1.0f / steinhart;                      // Invert
    steinhart -= 273.15f;                              // Convert to Celsius

    return steinhart;
  }


  float getTemperatureVoltage()
  {
    /*-------------------------------------------------------------------------
    Return the raw ADC voltage without any conversion
    -------------------------------------------------------------------------*/
    return counts_to_voltage( s_adc_samples[ CHANNEL_TEMP ] );
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
    return ( counts_to_voltage( s_adc_samples[ CHANNEL_VSUPPLY ] ) * ( R1 + R2 ) ) / R2;
  }


  float getMCUVoltage()
  {
    /*-------------------------------------------------------------------------
    Resistor Divider
    -------------------------------------------------------------------------*/
    constexpr float R1 = 10000.0f;
    constexpr float R2 = 10000.0f;

    /*-------------------------------------------------------------------------
    Get the ADC reading
    -------------------------------------------------------------------------*/
    return ( counts_to_voltage( s_adc_samples[ CHANNEL_VMCU ] ) * ( R1 + R2 ) ) / R2;
  }


  float getCurrentSenseReferenceVoltage()
  {
    return counts_to_voltage( s_adc_samples[ CHANNEL_VREF ] );
  }


  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  void Private::isr_on_instrumentation_adc_conversion_complete( const Chimera::ADC::InterruptDetail &isr )
  {
    /*-------------------------------------------------------------------------
    Update our notion of the ADC configuration
    -------------------------------------------------------------------------*/
    s_adc_vref = isr.vref;
    s_adc_vres = isr.resolution;

    /*-------------------------------------------------------------------------
    Copy the ADC samples into the module buffer
    -------------------------------------------------------------------------*/
    for( auto i = 0; i < isr.num_samples; i++ )
    {
      s_adc_samples[ i ] = isr.samples[ i ];
    }
  }

}    // namespace Orbit::Instrumentation
