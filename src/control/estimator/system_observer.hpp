/******************************************************************************
 *  File Name:
 *    system_observer.hpp
 *
 *  Description:
 *    Observer interface for FOC control
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CONTROL_SYSTEM_OBSERVER_HPP
#define ORBIT_CONTROL_SYSTEM_OBSERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstring>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ObserverState
  {
    float x1;
    float x2;
    float lambda_est; /**< Estimate of flux linkage of motor */
    float i_alpha_last;
    float i_beta_last;

    void clear()
    {
      x1           = 0.0f;
      x2           = 0.0f;
      lambda_est   = 0.0f;
      i_alpha_last = 0.0f;
      i_beta_last  = 0.0f;
    }
  };

  struct MotorState
  {
    float foc_motor_flux_linkage;
    float foc_motor_ld_lq_diff;
    float m_gamma_now;          /**< Observer gain */
    float m_phase_now_observer; /**< Observer phase */

    void clear()
    {
      foc_motor_flux_linkage = 0.015f;    // No clue if these are even close
      foc_motor_ld_lq_diff   = 0.0f;
      m_gamma_now            = 500.0f;    // No clue if this is even close
      m_phase_now_observer   = 0.0f;
    }
  };


  struct SpeedPosObserverState
  {
    float acc_theta_delta; /**< Accumulated theta delta */
    float lpf_alpha;       /**< Low pass filter alpha value */
    float omega_est;       /**< Estimated rotor speed */
    float omega_filtered;  /**< Estimated rotor speed (filtered) */
    float theta_comp;      /**< Estimated rotor angle (compensated) */
    float theta_est;       /**< Estimated rotor angle */

    void clear()
    {
      acc_theta_delta = 0.0f;
      lpf_alpha       = 0.0f;
      omega_est       = 0.0f;
      omega_filtered  = 0.0f;
      theta_comp      = 0.0f;
      theta_est       = 0.0f;
    }
  };


  /**
   * @brief Stores state for the current observer detailed in AN1078
   *
   * This is a state observer that estimates the current flowing through the
   * motor phases. It is used to estimate the back EMF voltage and correct
   * the output of the current controller.
   *
   * This operates on the motor currents after the Clarke transform.
   */
  struct CurrentObserverState
  {
    struct AlgVars
    {
      float  comp_z;         /* Output correction factor voltage */
      float  err_i;          /* Error between measured and estimated current */
      float  est_e;          /* Estimated back EMF */
      float  est_e_filtered; /* Estimated back EMF after filtering */
      float  est_i;          /* Estimated current */
      float  lpf_alpha;      /* Low pass filter alpha value */
      float *p_act_i;        /* Pointer to actual current measurement */
      float *p_cmd_v;        /* Pointer to actual voltage command */
    };

    float   f_gain;     /* Motor parameter based gain tunings */
    float   g_gain;     /* Motor parameter based gain tunings */
    AlgVars phase[ 2 ]; /* Alpha/Beta current observer state */

    void clear()
    {
      f_gain = 0.0f;
      g_gain = 0.0f;
      memset( phase, 0, sizeof( phase ) );
    }
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes all the system observers
   * @return void
   */
  void initObservers();

  /**
   * @brief Updates the current observer with the latest measurements
   *
   * Follows the decision tree presented in Figure 17 of AN1078
   */
  void updateCurrentObserver();


  void updateSpeedPosObserver();

  // TODO: Needs a way better name...
  void observerUpdate();

}    // namespace Orbit::Control

#endif /* !ORBIT_CONTROL_SYSTEM_OBSERVER_HPP */
