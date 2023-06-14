/******************************************************************************
 *  File Name:
 *    pid.hpp
 *
 *  Description:
 *    Basic PID controller implementation
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_PID_HPP
#define ORBIT_PID_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <algorithm>
#include <limits>

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

    PID() :
        SetPoint( 0 ), Output( 0 ), OutMinLimit( 0 ), OutMaxLimit( 0 ), Kp( 0 ), Ki( 0 ), Kd( 0 ), iTerm( 0 ), lastInput( 0 )
    {
    }

    void init()
    {
      SetPoint    = 0.0f;
      Output      = 0.0f;
      OutMinLimit = 0.0f;
      OutMaxLimit = 0.0f;
      Kp          = 0.0f;
      Ki          = 0.0f;
      Kd          = 0.0f;
      iTerm       = 0.0f;
      lastInput   = 0.0f;
    }

    /**
     * @brief Update the controller tunings accounting for discrete sampling
     *
     * @param kp  Proportional controller value
     * @param ki  Integral controller value
     * @param kd  Derivative controller value
     * @param dt  Sample time in seconds
     */
    void setTunings( const float kp, const float ki, const float kd, const float dt )
    {
      this->Kp = kp;
      this->Ki = ki * dt;
      this->Kd = kd / dt;
    }

    /**
     * @brief Runs the PID controller
     *
     * @param input   New input to inject
     * @param dt      Last sample delta
     */
    float run( const float input )
    {
      const float error = SetPoint - input;

      /*-----------------------------------------------------------------------
      Proportional Term
      -----------------------------------------------------------------------*/
      const float pTerm = Kp * error;

      /*-----------------------------------------------------------------------
      Integral Term w/Anti-Windup
      -----------------------------------------------------------------------*/
      iTerm += Ki * error;
      iTerm = std::max( OutMinLimit, std::min( iTerm, OutMaxLimit ) );

      /*-----------------------------------------------------------------------
      Derivative Term w/Anti-Kick
      -----------------------------------------------------------------------*/
      float dTerm = Kd * ( input - lastInput );

      /*-----------------------------------------------------------------------
      Calculate the output and save off state for the next run
      -----------------------------------------------------------------------*/
      Output    = pTerm + iTerm + dTerm;
      Output    = std::max( OutMinLimit, std::min( Output, OutMaxLimit ) );
      lastInput = input;

      return Output;
    }

    void resetState()
    {
      Output = 0.0f;
      lastInput = 0.0f;
      iTerm = 0.0f;
    }

  private:
    float Kp;
    float Ki;
    float Kd;
    float iTerm;
    float lastInput;
  };
}    // namespace Orbit::Control::Math

#endif /* !ORBIT_PID_HPP */
