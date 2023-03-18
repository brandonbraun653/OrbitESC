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
#include <src/control/filter.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/pid.hpp>
#include <etl/circular_buffer.h>

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
  using AlgoHistVal = etl::circular_buffer<float, 2>;

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
    ADC_CH_MOTOR_PHASE_C_CURRENT,
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

  struct ControllerData
  {
    /*-------------------------------------------------------------------------
    General Controller Data
    -------------------------------------------------------------------------*/
    volatile bool isrCtlActive;           /**< Enable/disable the controller update routines inside ADC ISR */
    float         last_current_update_us; /**< Last update in */
    float         last_estimate_update_us;
    float         next_estimate_update_us;


    Math::TrapInt IqrInt; /**< Q-axis current reference command integrator */
    Math::TrapInt SpdInt; /**< Speed estimation integrator (generates position) */

    float                Idm;  /**< D-axis measured current */
    float                Idf;  /**< D-axis filtered current measurement */
    float                Idr;  /**< D-axis current reference command */
    float                Vdr;  /**< D-axis commanded output voltage (phase) */
    Math::PID            Dpid; /**< D-axis pid controller */
    Math::FIR<float, 15> DFIR; /**< D-axis FIR filter for current measurement */

    float                Iqm;  /**< Q-axis measured current */
    float                Iqf;  /**< Q-axis filtered current measurement */
    float                Iqr;  /**< Q-axis current reference command */
    float                Vqr;  /**< Q-axis commanded output voltage (magnitude) */
    Math::PID            Qpid; /**< Q-axis pid controller */
    Math::FIR<float, 15> QFIR; /**< Q-axis FIR filter for current measurement */

    /*-------------------------------------------------------------------------
    Fast Loop Update Variables
    -------------------------------------------------------------------------*/
    float   svpwm_a_duty; /**< Duty cycle for phase A */
    float   svpwm_b_duty; /**< Duty cycle for phase B */
    float   svpwm_c_duty; /**< Duty cycle for phase C */
    uint8_t svpwm_comm;   /**< Commutation sector to use */

    /*-------------------------------------------------------------------------
    Estimated Quantities
    -------------------------------------------------------------------------*/
    float posEst;
    float spdEst;

    /*-------------------------------------------------------------------------
    Measured Quantities
    -------------------------------------------------------------------------*/
    float Vdd; /**< Supply voltage in volts */

    void clear()
    {
      IqrInt.reset();
      SpdInt.reset();

      posEst = 0.0f;
      spdEst = 0.0f;
      Vdd    = 0.0f;
      Idm = 0.0f;
      Idf = 0.0f;
      Idr = 0.0f;
      Vdr = 0.0f;
      Dpid.init();
      DFIR = Math::FIR<float, 15>();

      Iqm = 0.0f;
      Iqf = 0.0f;
      Iqr = 0.0f;
      Vqr = 0.0f;
      Qpid.init();
      QFIR = Math::FIR<float, 15>();

      svpwm_a_duty = 0.0f;
      svpwm_b_duty = 0.0f;
      svpwm_c_duty = 0.0f;
      svpwm_comm = 0;
    }
  };


  class ADCControl
  {
  public:
    struct ChannelData
    {
      float                measured;   /**< Measured ADC value, accounting for offset */
      float                calOffset;  /**< The DC offset of the ADC channel at idle */
      size_t               calSamples; /**< How many samples have been acquired */
      float                calSum;     /**< Total sum of the samples */
      Math::FIR<float, 15> lpFilter;   /**< Low pass filter for the ADC channel */

      void updateFilter( const Math::FIR<float, 15> &filter )
      {
        lpFilter = filter;
        lpFilter.initialize( 0.0f );
      }

      void clear()
      {
        measured  = 0.0f;
        calOffset = 0.0f;
        lpFilter.initialize( 0.0f );
      }
    };

    using ChannelBuffer = etl::array<ChannelData, ADC_CH_NUM_OPTIONS>;


    bool          calibrating;  /**< Flag to enable/disable calibration routines */
    size_t        calStartTime; /**< System time when the calibration started */
    size_t        sampleTimeUs; /**< Last time the data was sampled */
    ChannelBuffer data;

    void startCalibration()
    {
      if( !calibrating )
      {
        for( auto &ch : data )
        {
          ch.calSamples = 0;
          ch.calSum     = 0.0f;
        }

        calStartTime = Chimera::millis();
        calibrating  = true;
      }
    }

    void clear()
    {
      calibrating = false;
      for ( auto &ch : data )
      {
        ch.clear();
      }
    }
  };

  /**
   * @brief Collection of objects that represent the entire state of the FOC system
   * @note Yes, this is a god class. I know. I'm sorry.
   */
  struct SuperState
  {
    ADCControl      adc;
    EMFObserver     emfObserver;
    OmegaEstimator  speedEstimator;
    MotorParameters motorParams;
    ControllerData  motorCtl;

    void clear()
    {
      adc.clear();
      emfObserver.clear();
      speedEstimator.clear();
      motorParams.clear();
      motorCtl.clear();
    }
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_FOC_DATA_HPP */
