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
  }


  void RotorRamp::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );

    /*-----------------------------------------------------------------------------
    Reinitialize the current control loop
    -----------------------------------------------------------------------------*/

    Field::setControlMode( Field::Mode::OPEN_LOOP );
    Field::setInnerLoopReferences( 0.1f, 0.0f, 0.0f );

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
  }


  State RotorRamp::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
