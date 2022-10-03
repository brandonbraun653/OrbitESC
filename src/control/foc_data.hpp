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

  /**
   * @brief Stores the current and last value of a number
   *
   * Intended for use with the discrete controllers that perform calculations
   * on results from the previous iteration.
   */
  using AlgoHistVal = std::array<float, 2>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

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

  enum AlgoHistIdx
  {
    HIST_IDX_CURR = 0,
    HIST_IDX_LAST = 1,

    HIST_IDX_NUM_OPTIONS,
    HIST_IDX_INVALID
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

  struct ParkControl
  {
    uint32_t startTime_ms;        /**< When the park controller started executing */
    uint32_t lastUpdate_ms;       /**< Last time the controller was updated */
    uint32_t activeComState;      /**< Which commutation state to align the rotor on */
    uint32_t alignTime_ms;        /**< How long to run the park mode for in milliseconds */
    uint32_t modulation_dt_ms;    /**< dt to toggle the output state in milliseconds */
    float    phaseDutyCycle[ 3 ]; /**< Drive strength for each phase */
    bool     outputEnabled;       /**< Enable/disable the power stage */

    void clear()
    {
      memset( this, 0, sizeof( ParkControl ) );
    }
  };

  struct RampControl
  {
    /*-------------------------------------------------------------------
    Input Variables
    -------------------------------------------------------------------*/
    uint32_t rampRate;       /**< Ramp rate in RPM/update */
    uint32_t finalRPM;       /**< End RPM value to achieve when ramp control is finished */
    uint32_t targetRPM;      /**< Current desired RPM */
    uint32_t minDwellCycles; /**< Minimum cycle events before target RPM can update */

    /*-------------------------------------------------------------------
    Working Variables (used inside ISR)
    -------------------------------------------------------------------*/
    uint32_t cycleCount; /**< How many times the inner loop ISR has been hit */
    uint32_t cycleRef;   /**< Threshold for when to trigger a commutation update */
    uint32_t currentRPM;

    /*-------------------------------------------------------------------
    Output Variables
    -------------------------------------------------------------------*/
    uint32_t comState;            /**< Current commutation state to drive */
    float    phaseDutyCycle[ 3 ]; /**< Drive strength for each phase */

    void clear()
    {
      memset( this, 0, sizeof( RampControl ) );
    }
  };

  struct RunControl
  {
    /**
     * Input Variables
     */
    float speedRefRad; /**< Reference speed in radians/second */


    void clear()
    {
      memset( this, 0, sizeof( RunControl ) );
    }
  };

  struct ControllerData
  {
    /*-------------------------------------------------------------------------
    General Controller Data
    -------------------------------------------------------------------------*/
    volatile bool isrCtlActive;    /**< Enable/disable the controller update routines inside ADC ISR */
    float last_current_update_us;  /**< Last update in */
    float last_estimate_update_us;
    float next_estimate_update_us;

    /*-------------------------------------------------------------------------
    Estimated Quantities
    -------------------------------------------------------------------------*/
    AlgoHistVal posEstRad; /**< Position in radians */
    AlgoHistVal velEstRad; /**< Velocity in radians/second */
    AlgoHistVal accEstRad; /**< Acceleration in radians/second^2 */

    /*-------------------------------------------------------------------------
    Measured Quantities
    -------------------------------------------------------------------------*/
    float Vdd; /**< Supply voltage in volts */
    float Id;  /**< Current in Amps */
    float Iq;  /**< Current in Amps */

    /*-------------------------------------------------------------------------
    Output Variables
    -------------------------------------------------------------------------*/
    float Vd; /**< Commanded output voltage on the D axis */
    float Vq; /**< Commanded output voltage on the Q axis */


    void clear()
    {
      posEstRad.fill( 0.0f );
      velEstRad.fill( 0.0f );
      accEstRad.fill( 0.0f );
      Vdd = 0.0f;
      Id  = 0.0f;
      Iq  = 0.0f;
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
    ControllerData  motorCtl;

    void clear()
    {
      for ( auto &data : adcBuffer )
      {
        data.clear();
      }

      emfObserver.clear();
      speedEstimator.clear();
      motorParams.clear();
      motorCtl.clear();
    }
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_FOC_DATA_HPP */
