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
#include <src/control/subroutines/declarations.hpp>

namespace Orbit::Control::Subroutine
{
  /*-----------------------------------------------------------------------------
  RotorDetector Implementation
  -----------------------------------------------------------------------------*/
  RotorDetector::RotorDetector()
  {
    id     = Routine::ALIGNMENT_DETECTION;
    mState = State::UNINITIALIZED;
  }

  RotorDetector::~RotorDetector()
  {
  }


  void RotorDetector::initialize()
  {
    mState = State::INITIALIZED;
  }


  void RotorDetector::start()
  {
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
