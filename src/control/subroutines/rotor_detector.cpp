/******************************************************************************
 *  File Name:
 *    rotor_detector.cpp
 *
 *  Description:
 *    Subroutine for detecting the static position of the rotor
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/control/subroutines/declarations.hpp>

namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  RotorDetector Implementation
  ---------------------------------------------------------------------------*/

  RotorDetector::RotorDetector()
  {
    id     = Routine::ALIGNMENT_DETECTION;
    name   = "Rotor Alignment Detector";
    mState = State::UNINITIALIZED;
  }


  RotorDetector::~RotorDetector()
  {
  }


  void RotorDetector::initialize()
  {
    LOG_INFO( "Initialized rotor detector" );
    mState = State::INITIALIZED;
  }


  void RotorDetector::start()
  {
    LOG_INFO( "Running rotor detector" );
    mState = State::RUNNING;
  }


  void RotorDetector::stop()
  {
    mState = State::STOPPED;
  }


  void RotorDetector::destroy()
  {
    mState = State::UNINITIALIZED;
  }


  void RotorDetector::process()
  {
  }


  State RotorDetector::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
