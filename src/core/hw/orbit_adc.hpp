/******************************************************************************
 *  File Name:
 *    orbit_adc.hpp
 *
 *  Description:
 *    Orbit ESC ADC driver interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ADC_HPP
#define ORBIT_ESC_ADC_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>


namespace Orbit::ADC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the ADC driver subsystem
   */
  void powerUp();

  /**
   * @brief Transfer function to convert ADC voltage to measured phase current
   *
   * @param vin     Measurement voltage
   * @return float  Calculated phase current in Amps
   */
  float sample2PhaseCurrent( const float vin );

  /**
   * @brief Transfer function to convert ADC voltage to measured bus voltage
   *
   * @param vin     Measurement voltage
   * @return float  Calculated bus voltage
   */
  float sample2BusVoltage( const float vin );

  /**
   * @brief Gets the voltage reference driving the phase current sense amplifiers
   * @return float
   */
  float pIxVRef();

}  // namespace Orbit::ADC

#endif  /* !ORBIT_ESC_ADC_HPP */
