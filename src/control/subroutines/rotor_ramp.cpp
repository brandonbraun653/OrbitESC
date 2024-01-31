/******************************************************************************
 *  File Name:
 *    rotor_ramp.cpp
 *
 *  Description:
 *    Implementation of the rotor ramping subroutine
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/control/subroutines/rotor_ramp.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>


namespace Orbit::Control::Subroutine
{

  // TODO: If the system needs to bail mid-ramp, post an event to the FOC::sendSystemEvent method.

  /*---------------------------------------------------------------------------
  RotorRamp Implementation
  ---------------------------------------------------------------------------*/

  RotorRamp::RotorRamp()
  {
  }


  RotorRamp::~RotorRamp()
  {
  }


  void RotorRamp::initialize()
  {
    LOG_INFO( "Initialized %s", this->name.c_str() );
    mState = State::INITIALIZED;
  }


  void RotorRamp::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );
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

}  // namespace Orbit::Control::Subroutine
