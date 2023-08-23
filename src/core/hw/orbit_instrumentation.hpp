/******************************************************************************
 *  File Name:
 *    orbit_instrumentation.hpp
 *
 *  Description:
 *    OrbitESC interface for reading various low level sensor measurements on
 *    the board. To-date this has been used for temperature and voltage
 *    measurements of interest.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SENSOR_HPP
#define ORBIT_ESC_SENSOR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::Instrumentation
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Ordering of the ADC channels in the sample buffer.
   *
   * The sampling order of the ADC is used in several places for both
   * configuring and interpreting data, so it's convenient to define it once.
   */
  enum ChannelSequence : uint8_t
  {
    CHANNEL_TEMP,    /**< Board PCB temperature measurement */
    CHANNEL_VSUPPLY, /**< High power ESC supply voltage */
    CHANNEL_VMCU,    /**< Input voltage to the MCU */
    CHANNEL_VREF,    /**< Reference voltage to motor phase current op-amps */

    CHANNEL_COUNT
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Powers up the sensor driver subsystem.
   *
   * Configures the ADC and timer peripherals for use with the sensor driver.
   * Upon exit, measurements will be available in the sample buffer.
   *
   * @return void
   */
  void powerUp();

  /**
   * @brief Gets the board temperature in degrees Celcius
   * @return float
   */
  float getTemperatureCelcius();

  /**
   * @brief Gets the current supply voltage in volts
   * @return float
   */
  float getSupplyVoltage();

  /**
   * @brief Gets the current MCU voltage in volts
   * @return float
   */
  float getMCUVoltage();

  /**
   * @brief Get the voltage used as a reference for current sense amplifiers
   * @return float
   */
  float getCurrentSenseReferenceVoltage();

}    // namespace Orbit::Instrumentation

#endif /* !ORBIT_ESC_SENSOR_HPP */
