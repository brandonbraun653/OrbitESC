/******************************************************************************
 *  File Name:
 *    rotor_ramp.cpp
 *
 *  Description:
 *    Implementation of the rotor ramping subroutine. This is attempting to
 *    follow the description from the following white paper (Section 3):
 *
 *  https://scolton-www.s3.amazonaws.com/motordrive/sensorless_gen1_Rev1.pdf
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/control/hardware/current_control.hpp>
#include <src/control/subroutines/rotor_ramp.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/control/foc_math.hpp>


namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  RotorRamp Implementation
  ---------------------------------------------------------------------------*/

  RotorRamp::RotorRamp()
  {
    id     = Routine::OPEN_LOOP_RAMP_FOC;
    name   = "FOC Rotor Ramp";
    mState = RunState::UNINITIALIZED;
  }


  RotorRamp::~RotorRamp()
  {
  }


  void RotorRamp::initialize()
  {
    LOG_INFO( "Initialized %s", this->name.c_str() );
    mState = RunState::INITIALIZED;

    /*-------------------------------------------------------------------------
    Reset the current control loop
    -------------------------------------------------------------------------*/
    Field::powerDn();
    Field::powerUp();


  }


  void RotorRamp::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );

    /*-----------------------------------------------------------------------------
    Reinitialize the current control loop
    -----------------------------------------------------------------------------*/
    Field::setControlMode( Field::Mode::OPEN_LOOP );
    Field::setInnerLoopReferences( 0.0f, 0.0f, 0.0f );

    mState = RunState::RUNNING;
  }


  void RotorRamp::stop()
  {
    LOG_INFO( "Stopped %s", this->name.c_str() );
    mState = RunState::STOPPED;
  }


  void RotorRamp::destroy()
  {
    mState = RunState::UNINITIALIZED;
  }


  void RotorRamp::process()
  {
    using namespace Control::Math;

    static constexpr float a = 0.1f;
    static constexpr float b = 1.0f;
    static constexpr float c = 0.0;


    // if( ( Chimera::millis() - start_time ) < 10000 )
    // {
    //   float curr_time_us = static_cast<float>( Chimera::micros() ) / 1e6f;

    //   dt         = curr_time_us - start_time_us;
    //   theta_incr = ( a * dt * dt ) + ( b * dt ) + c;

    //   if( theta_incr > M_PI_F )
    //   {
    //     theta_incr = M_PI_F;
    //   }
    // }

    // theta_ref += theta_incr;
    // while( theta_ref > M_2PI_F )
    // {
    //   theta_ref -= M_2PI_F;
    // }

    // Field::setInnerLoopReferences( 0.1f, 0.0f, theta_ref );
  }


  RunState RotorRamp::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
