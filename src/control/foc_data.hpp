/******************************************************************************
 *  File Name:
 *    foc_data.hpp
 *
 *  Description:
 *    FOC data structures and utilities
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_FOC_DATA_HPP
#define ORBIT_FOC_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <array>
#include <cstdint>
#include <Chimera/common>
#include <Chimera/adc>
#include <Chimera/timer>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  /**
   * @brief Function signature for a transfer function to convert ADC values to useful
   *
   * @param[in] vin   Input voltage measured by the ADC
   * @return float    Output value of an arbitrary unit
   */
  using ADCTxfrFunc = float ( * )( float vin );

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum ADCChannel
  {
    ADC_CH_MOTOR_PHASE_A_CURRENT,
    ADC_CH_MOTOR_PHASE_B_CURRENT,
    ADC_CH_MOTOR_SUPPLY_VOLTAGE,

    ADC_CH_NUM_OPTIONS,
    ADC_CH_INVALID
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct EMFObserver
  {
    float d; /**< Observer eigenvalue selection */
    float z1;
    float z1_dot;
    float z2;
    float z2_dot;
    float Ed_est;         /**< Estimated back-EMF on the D axis */
    float Eq_est;         /**< Estimated back-EMF on the Q axis */
    float last_update_us; /**< Time of last update in microseconds */

    void clear()
    {
      z1     = 0.0f;
      z1_dot = 0.0f;
      z2     = 0.0f;
      z2_dot = 0.0f;
    }
  };

  struct OmegaEstimator
  {
    float pos_est_prev;   /**< Last known rotor position in radians */
    float last_update_us; /**< Time of last update in microseconds */

    void clear()
    {
      pos_est_prev   = 0.0f;
      last_update_us = 0.0f;
    }
  };

  struct MotorParameters
  {
    float Ls; /**< Stator inductance in Henrys */
    float Rs; /**< Stator resistance in Ohms */

    void clear()
    {
      Ls = 0.0f;
      Rs = 0.0f;
    }
  };

  struct ControllerData
  {
    /*-------------------------------------------------------------------------
    Estimated Quantities
    -------------------------------------------------------------------------*/
    float posEstRad; /**< Position in radians */
    float velEstRad; /**< Velocity in radians/second */
    float accEstRad; /**< Acceleration in radians/second^2 */

    /*-------------------------------------------------------------------------
    Measured Quantities
    -------------------------------------------------------------------------*/
    float Vdd; /**< Supply voltage in volts */
    float Id;  /**< Current in Amps */
    float Iq;  /**< Current in Amps */

    /*-------------------------------------------------------------------------
    Controller Outputs
    -------------------------------------------------------------------------*/
    float Vd; /**< Commanded output voltage on the D axis */
    float Vq; /**< Commanded output voltage on the Q axis */

    void clear()
    {
      posEstRad = 0.0f;
      velEstRad = 0.0f;
      accEstRad = 0.0f;
      Vdd       = 0.0f;
      Id        = 0.0f;
      Iq        = 0.0f;
      Vd        = 0.0f;
      Vq        = 0.0f;
    }
  };

  struct ADCSensorData
  {
    float    measured;     /**< Raw ADC value */
    float    converted;    /**< Converted raw value into meaningful data */
    float    dcOffset;     /**< The DC offset of the ADC channel */
    uint32_t sampleTimeUs; /**< The time in microseconds that the ADC sample was taken */

    void clear()
    {
      measured     = 0.0f;
      converted    = 0.0f;
      dcOffset     = 0.0f;
      sampleTimeUs = 0;
    }
  };

  using ADCSensorBuffer = std::array<ADCSensorData, ADC_CH_NUM_OPTIONS>;


  /**
   * @brief Collection of objects that represent the entire state of the FOC system
   * @note Yes, this is a god class. I know. I'm sorry.
   */
  struct SuperState
  {
    ADCSensorBuffer adcBuffer;
    EMFObserver     emfObserver;
    OmegaEstimator  speedEstimator;
    MotorParameters motorParams;
    ControllerData  motorState;

    void clear()
    {
      for ( auto &data : adcBuffer )
      {
        data.clear();
      }

      emfObserver.clear();
      speedEstimator.clear();
      motorParams.clear();
      motorState.clear();
    }
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_FOC_DATA_HPP */
