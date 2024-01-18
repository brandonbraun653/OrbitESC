/******************************************************************************
 *  File Name:
 *    rotor_detector.hpp
 *
 *  Description:
 *    Subroutine for detecting the static position of the rotor
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ROTOR_DETECTOR_SUBROUTINE_HPP
#define ORBIT_ESC_ROTOR_DETECTOR_SUBROUTINE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <etl/array.h>
#include <src/control/subroutines/interface.hpp>


namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct RDMeasurement
  {
    Chimera::Timer::Inverter::SwitchIO hiSide;
    Chimera::Timer::Inverter::SwitchIO loSide;
    float                              accCurrent;
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Subroutine for detecting the static rotor position.
   *
   * Only intended to be used for startup algorithms that need to detect
   * the initial position of the rotor for driving the stator properly.
   */
  class RotorDetector : public ISubroutine
  {
  public:
    RotorDetector();
    ~RotorDetector();

    void  initialize() final override;
    void  start() final override;
    void  stop() final override;
    void  destroy() final override;
    void  process() final override;
    State state() final override;

  private:
    Chimera::Timer::Inverter::Driver *mTimer;
    etl::array<RDMeasurement, 6>      mMeasurements;
    size_t                            mStartTimeUs;
    uint32_t                          mIdx;
    bool                              mSampleActive;
  };

}    // namespace Orbit::Control::Subroutine

#endif /* !ORBIT_ESC_ROTOR_DETECTOR_SUBROUTINE_HPP */
