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

  // TODO: If the system needs to bail mid-ramp, post an event to the FOC::sendSystemEvent method.
  /** Implementation Notes
   *
   * Ok. The important thing to remember here is that all control aspects are
   * occuring from the DQ frame only. There are HW timers, DMA engines, and
   * high frequency interrupts that are all running in the background to make
   * the current control loop functional.
   *
   * That being said, the first thing I probably should do is get that loop up
   * and running again. It needs a bit of polishing.
   *
   */

  static uint32_t start_time;
  static float theta_incr;
  static float theta_ref;
  static float start_time_us;
  static float dt;

  /*---------------------------------------------------------------------------
  RotorRamp Implementation
  ---------------------------------------------------------------------------*/

  RotorRamp::RotorRamp()
  {
    id     = Routine::OPEN_LOOP_RAMP_FOC;
    name   = "OL Rotor Ramp FOC";
    mState = State::UNINITIALIZED;
  }


  RotorRamp::~RotorRamp()
  {
  }


  void RotorRamp::initialize()
  {
    LOG_INFO( "Initialized %s", this->name.c_str() );
    mState = State::INITIALIZED;

    /*-------------------------------------------------------------------------
    Reset the current control loop
    -------------------------------------------------------------------------*/
    Field::powerDn();
    Field::powerUp();

    theta_ref = 0.0f;
    theta_incr = 0.0f;
    start_time_us = 0.0f;
    dt = 0.0f;
  }


  void RotorRamp::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );

    /*-----------------------------------------------------------------------------
    Reinitialize the current control loop
    -----------------------------------------------------------------------------*/
    Field::setControlMode( Field::Mode::OPEN_LOOP );
    Field::setInnerLoopReferences( 0.1f, 0.0f, theta_ref );

    start_time_us = static_cast<float>( Chimera::micros() ) / 1e6f;
    start_time = Chimera::millis();

    mState = State::RUNNING;
  }


  void RotorRamp::stop()
  {
    LOG_INFO( "Stopped %s", this->name.c_str() );
    mState = State::STOPPED;
  }


  void RotorRamp::destroy()
  {
    mState = State::UNINITIALIZED;
  }


  void RotorRamp::process()
  {
    using namespace Control::Math;

    static constexpr float a = 0.1f;
    static constexpr float b = 1.0f;
    static constexpr float c = 0.0;


    // if( ( Chimera::millis() - start_time ) < 10000 )
    // {
      float curr_time_us = static_cast<float>( Chimera::micros() ) / 1e6f;

      dt         = curr_time_us - start_time_us;
      theta_incr = ( a * dt * dt ) + ( b * dt ) + c;

      if( theta_incr > M_PI_F )
      {
        theta_incr = M_PI_F;
      }
    // }

    theta_ref += theta_incr;
    while( theta_ref > M_2PI_F )
    {
      theta_ref -= M_2PI_F;
    }

    Field::setInnerLoopReferences( 0.1f, 0.0f, theta_ref );
  }


  State RotorRamp::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
