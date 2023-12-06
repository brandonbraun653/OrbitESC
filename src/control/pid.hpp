/******************************************************************************
 *  File Name:
 *    pid.hpp
 *
 *  Description:
 *    Basic PID controller implementation
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_PID_HPP
#define ORBIT_PID_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::Control::Math
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class PID
  {
  public:
    float SetPoint;    /**< System control set point input*/
    float Output;      /**< PID controller output */
    float OutMinLimit; /**< Configurable minimum limit */
    float OutMaxLimit; /**< Configurable maximum limit */

    PID();

    /**
     * @brief Re-initializes the PID controller.
     * @note This destroys all the controller settings and state.
     * @return void
     */
    void init();

    /**
     * @brief Resets the state of the PID controller.
     * @note Only resets the state, not the PID settings.
     * @return void
     */
    void resetState();

    /**
     * @brief Update the controller tunings accounting for discrete sampling
     *
     * @param kp  Proportional controller value
     * @param ki  Integral controller value
     * @param kd  Derivative controller value
     * @param dt  Sample time in seconds
     * @return void
     */
    void setTunings( const float kp, const float ki, const float kd, const float dt );

    /**
     * @brief Runs the PID controller
     *
     * @param input   New input to inject
     * @param dt      Last sample delta
     * @return New output value
     */
    float run( const float input );

  private:
    float Kp;
    float Ki;
    float Kd;
    float iTerm;
    float lastInput;
  };
}    // namespace Orbit::Control::Math

#endif /* !ORBIT_PID_HPP */
