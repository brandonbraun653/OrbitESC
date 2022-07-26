/******************************************************************************
 *  File Name:
 *    drv8301.hpp
 *
 *  Description:
 *    Driver for the BOOSTXL-DRV8301.
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_BOOSTXL_DRV8301_HPP
#define ORBIT_BOOSTXL_DRV8301_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::BoostXL
{
  /*---------------------------------------------------------------------------
  Voltage Sense Constants
  ---------------------------------------------------------------------------*/
  static constexpr float VFULL_SCALE_DC_BUS = 26.314f;
  static constexpr float VREF_DC_BUS        = 3.3f;
  static constexpr float DC_BUS_V_PER_ADC_V = VFULL_SCALE_DC_BUS / VREF_DC_BUS;

  /*---------------------------------------------------------------------------
  Current Sense Constants
  ---------------------------------------------------------------------------*/
  static constexpr float IFULL_SCALE     = 16.5f * 2.0f;    // Section 5.2
  static constexpr float ISHUNT_AMP_GAIN = 10.0f;           // Configured gain of all amplifiers
  static constexpr float ISENSE_VREF     = 1.65f;           // Voltage reference for current sense
  static constexpr float RSHUNT_OHM      = 0.01f;           // Low-side shunt resistor

  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/

  static float adc_to_phase_current( float vin )
  {
    return ( vin - ISENSE_VREF ) / ( ISHUNT_AMP_GAIN * RSHUNT_OHM );
  }

  static float adc_to_bus_voltage( float vin )
  {
    return vin * DC_BUS_V_PER_ADC_V;
  }

}    // namespace Orbit::BoostXL

#endif /* !ORBIT_BOOSTXL_DRV8301_HPP */
