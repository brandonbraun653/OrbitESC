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
#include <src/control/math/filter.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/math/pid.hpp>
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

  /**
   * @brief Attributes describing the state of the physical system under control.
   */
  struct SystemState
  {
    /*-------------------------------------------------------------------------
    Measured Quantities
    -------------------------------------------------------------------------*/
    float theta; /**< Rotor electrical position in radians */
    float omega; /**< Rotor speed in radians per second */

    /*-------------------------------------------------------------------------
    Estimated Quantities
    -------------------------------------------------------------------------*/
    float thetaEst; /**< Rotor electrical position estimate in radians */
    float omegaEst; /**< Rotor speed estimate in radians per second */
  };

  extern SystemState foc_motor_state;


  /**
   * @brief State data for the inner current control loop
   */
  struct CurrentControlState
  {
    float              dt;    /**< Time delta between current control loop invocation */
    float              ima;   /**< Current measured at phase A terminal */
    float              imb;   /**< Current measured at phase B terminal */
    float              imc;   /**< Current measured at phase C terminal */
    float              vma;   /**< Voltage measured at phase A terminal */
    float              vmb;   /**< Voltage measured at phase B terminal */
    float              vmc;   /**< Voltage measured at phase C terminal */
    float              iq;    /**< Current output measurement for the q-axis */
    float              id;    /**< Current output measurement for the d-axis */
    float              vq;    /**< Voltage command for the q-axis */
    float              vd;    /**< Voltage command for the d-axis */
    float              va;    /**< Voltage (alpha) after inverse-park transform */
    float              vb;    /**< Voltage (beta) after inverse-park transform */
    float              ia;    /**< Current (alpha) after clarke transform */
    float              ib;    /**< Current (beta) after clarke transform */
    float              iqRef; /**< Current reference for the q-axis */
    float              idRef; /**< Current reference for the d-axis */
    Control::Math::PID iqPID; /**< Current controller for the q-axis */
    Control::Math::PID idPID; /**< Current controller for the d-axis */
  };

  extern CurrentControlState foc_ireg_state;


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
      Idm    = 0.0f;
      Idf    = 0.0f;
      Idr    = 0.0f;
      Vdr    = 0.0f;
      Dpid.init();
      DFIR = Math::FIR<float, 15>();

      Iqm = 0.0f;
      Iqf = 0.0f;
      Iqr = 0.0f;
      Vqr = 0.0f;
      Qpid.init();
      QFIR = Math::FIR<float, 15>();
    }
  };


  /**
   * @brief Collection of objects that represent the entire state of the FOC system
   * @note Yes, this is a god class. I know. I'm sorry.
   */
  struct SuperState
  {
    EMFObserver     emfObserver;
    OmegaEstimator  speedEstimator;
    MotorParameters motorParams;
    ControllerData  motorCtl;

    void clear()
    {
      emfObserver.clear();
      speedEstimator.clear();
      motorParams.clear();
      motorCtl.clear();
    }
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the global FOC data structures
   * @return void
   */
  void initFOCData();

}    // namespace Orbit::Control

#endif /* !ORBIT_FOC_DATA_HPP */
