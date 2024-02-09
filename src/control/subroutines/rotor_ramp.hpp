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
  Enumerations
  ---------------------------------------------------------------------------*/

  enum class RampStep : uint8_t
  {
    ALIGN,    /**< Aligning the rotor to a starting vector */
    RAMP,     /**< Controlled ramp from zero to idle speed */
    COMPLETE, /**< Rotor is at idle speed */
    ERROR     /**< Ramp subroutine failed */
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct RampState
  {
    RampStep rampStep;     /**< Current step in the ramping process */
    uint32_t startTimeRef; /**< Time reference for the start of a process */
    float    thetaRefRad;  /**< Reference angle (radians) to drive the FOC vector */
    float    iqRef;        /**< Reference current on the Q-axis to drive the FOC vector */
    float    idRef;        /**< Reference current on the D-axis to drive the FOC vector */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Subroutine for ramping rotor speed from zero to an idle value
   */
  class RotorRamp : public ISubroutine
  {
  public:
    RotorRamp();
    ~RotorRamp();

    void     initialize() final override;
    void     start() final override;
    void     stop() final override;
    void     destroy() final override;
    void     process() final override;
    RunState state() final override;

  private:
    RampState mRampState;
  };

}    // namespace Orbit::Control::Subroutine

#endif /* !ORBIT_ESC_ROTOR_RAMP_SUBROUTINE_HPP */
