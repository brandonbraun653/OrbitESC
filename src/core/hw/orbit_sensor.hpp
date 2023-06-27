/******************************************************************************
 *  File Name:
 *    orbit_sensor.hpp
 *
 *  Description:
 *    Sensor interface for the OrbitESC board
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SENSOR_HPP
#define ORBIT_ESC_SENSOR_HPP

namespace Orbit::Sensor
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the sensor driver subsystem
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

}  // namespace

#endif  /* !ORBIT_ESC_SENSOR_HPP */
