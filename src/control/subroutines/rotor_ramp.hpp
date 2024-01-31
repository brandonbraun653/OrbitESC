/******************************************************************************
 *  File Name:
 *    rotor_ramp.hpp
 *
 *  Description:
 *    Subroutine for ramping the motor from a standstill to a speed where the
 *    closed loop control can take over
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ROTOR_RAMP_SUBROUTINE_HPP
#define ORBIT_ESC_ROTOR_RAMP_SUBROUTINE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <etl/array.h>
#include <src/control/subroutines/interface.hpp>


namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Subroutine for detecting the static rotor position.
   *
   * Only intended to be used for startup algorithms that need to detect
   * the initial position of the rotor for driving the stator properly.
   */
  class RotorRamp : public ISubroutine
  {
  public:
    RotorRamp();
    ~RotorRamp();

    void  initialize() final override;
    void  start() final override;
    void  stop() final override;
    void  destroy() final override;
    void  process() final override;
    State state() final override;

  private:
  };

}    // namespace Orbit::Control::Subroutine

#endif /* !ORBIT_ESC_ROTOR_RAMP_SUBROUTINE_HPP */
