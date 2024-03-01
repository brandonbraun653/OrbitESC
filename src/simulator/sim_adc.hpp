/******************************************************************************
 *  File Name:
 *    sim_adc.hpp
 *
 *  Description:
 *    ADC simulation model for the test harness
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ADC_SIMULATOR_HPP
#define ORBIT_ESC_ADC_SIMULATOR_HPP

namespace Orbit::Sim::ADC
{
  /**
   * @brief Initialize the virtual ADC module
   *
   * @param vref  Reference voltage for the ADC
   * @param vres  Resolution of the ADC in volts per bit
   * @return void
   */
  void initialize( const float vref, const float vres );

  /**
   * @brief Enable/disable simulation periodic triggering of the ADC
   *
   * @param enable  True to enable periodic triggering, false to disable
   * @return void
   */
  void enableMotorSenseADC( const bool enable );

  /**
   * @brief Sets the next phase voltage readings for the ADC
   *
   * These will be internally converted to the correct "counts" values
   *
   * @param va  Actual voltage on phase A
   * @param vb  Actual voltage on phase B
   * @param vc  Actual voltage on phase C
   * @return void
   */
  void setPhaseVoltage( const float va, const float vb, const float vc );

  /**
   * @brief Sets the next phase current readings for the ADC
   *
   * These will be internally converted to the correct "counts" values
   *
   * @param ia  Actual current on phase A
   * @param ib  Actual current on phase B
   * @param ic  Actual current on phase C
   * @return void
   */
  void setPhaseCurrent( const float ia, const float ib, const float ic );


  /**
   * @brief Sets the next DC bus voltage reading for the ADC
   *
   * This will be internally converted to the correct "counts" value
   *
   * @param vbus  Actual voltage on the DC bus
   * @return void
   */
  void setDCBusVoltage( const float vbus );

  /**
   * @brief Triggers a sample event for the motor sense ADC.
   *
   * This simulates an interrupt driven event that would occur in the real
   * hardware. The ADC will be sampled and the results will be available
   * for reading.
   *
   * @return void
   */
  void triggerMotorSenseADC();

  /**
   * @brief Triggers a sample event for the instrumentation ADC.
   *
   * This simulates an interrupt driven event that would occur in the real
   * hardware. The ADC will be sampled and the results will be available
   * for reading.
   *
   * @return void
   */
  void triggerInstrumentationADC();
}  // namespace Orbit::Sim::ADC

#endif  /* !ORBIT_ESC_ADC_SIMULATOR_HPP */
