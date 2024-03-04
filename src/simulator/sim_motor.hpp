/******************************************************************************
 *  File Name:
 *    sim_motor.hpp
 *
 *  Description:
 *    Motor simulation model for the test harness
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_MOTOR_SIMULATOR_HPP
#define ORBIT_ESC_MOTOR_SIMULATOR_HPP

namespace Orbit::Sim::Motor
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Static attributes of the motor model
   */
  struct Parameters
  {
    float J;          /**< Rotor/Load Inertia in Nm*s^2 */
    float v_max_adc;  /**< Max voltage that ADC can measure */
    int   pole_pairs; /**< Number of pole pairs ( pole numbers / 2) */
    float km;         /**< Constant = 1.5 * pole pairs */
    float ld;         /**< Motor inductance in D axis in uHy */
    float lq;         /**< Motor inductance in Q axis in uHy */
    float r;          /**< Motor resistance in ohms */
    float lpm;        /**< Flux linkage (lambda) in permanent magnets */
  };

  /**
   * @brief Computed state of the motor model
   */
  struct State
  {
    bool  connected; /**< True => connected; False => disconnected */
    float Ts;        /**< Time delta between samples in seconds */
    float id;        /**< Current in D-Direction in Amps */
    float iq;        /**< Current in Q-Direction in A */
    float me;        /**< Electrical Torque in Nm */
    float we;        /**< Electrical Angular Velocity in Rad/s */
    float phi;       /**< Electrical Rotor Angle in Rad */
    float sin_phi;   /**< Sine of Phi */
    float cos_phi;   /**< Cosine of Phi */
    float tsj;       /**< Ts / J */
    float ml;        /**< Load torque */
    float v_alpha;   /**< Alpha axis voltage in Volts */
    float v_beta;    /**< Beta axis voltage in Volts */
    float va;        /**< Phase a voltage in Volts */
    float vb;        /**< Phase b voltage in Volts */
    float vc;        /**< Phase c voltage in Volts */
    float vd;        /**< D axis voltage in Volts */
    float vq;        /**< Q axis voltage in Volts */
    float i_alpha;   /**< Alpha axis current in Amps */
    float i_beta;    /**< Beta axis current in Amps */
    float ia;        /**< Phase a current in Amps */
    float ib;        /**< Phase b current in Amps */
    float ic;        /**< Phase c current in Amps */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Attaches the motor model and begins simulation
   * @return void
   */
  void connect( const Parameters &params );

  /**
   * @brief Detaches the motor model and stops simulation
   * @return void
   */
  void disconnect();

  /**
   * @brief Steps the model forward using commanded alpha and beta voltages
   *
   * @param alpha Voltage in the alpha direction
   * @param beta  Voltage in the beta direction
   */
  void stepModel( const float alpha, const float beta );

  /**
   * @brief Gets a copy of the current motor state
   * @return MotorState
   */
  State modelState();

}    // namespace Orbit::Sim::Motor

#endif /* !ORBIT_ESC_MOTOR_SIMULATOR_HPP */
