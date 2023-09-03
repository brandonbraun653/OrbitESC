/******************************************************************************
 *  File Name:
 *    pid.cpp
 *
 *  Description:
 *    PID controller implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <algorithm>
#include <limits>
#include <src/control/pid.hpp>

namespace Orbit::Control::Math
{
  PID::PID() :
      SetPoint( 0 ), Output( 0 ), OutMinLimit( 0 ), OutMaxLimit( 0 ), Kp( 0 ), Ki( 0 ), Kd( 0 ), iTerm( 0 ), lastInput( 0 )
  {
  }

  void PID::init()
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


  void PID::setTunings( const float kp, const float ki, const float kd, const float dt )
  {
    this->Kp = kp;
    this->Ki = ki * dt;
    this->Kd = kd / dt;
  }


  void PID::resetState()
  {
    Output    = 0.0f;
    lastInput = 0.0f;
    iTerm     = 0.0f;
  }


  float PID::run( const float input )
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
}    // namespace Orbit::Control::Math
