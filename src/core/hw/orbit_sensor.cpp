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
  static Chimera::ADC::Driver_rPtr s_adc_driver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    s_adc_driver = Chimera::ADC::getDriver( IO::Analog::SensorPeripheral );
    RT_DBG_ASSERT( s_adc_driver );
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
