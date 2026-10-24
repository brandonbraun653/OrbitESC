/******************************************************************************
 *  File Name:
 *    rotor_align.hpp
 *
 *  Description:
 *    Subroutine for aligning the rotor to a known position
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ROTOR_ALIGN_SUBROUTINE_HPP
#define ORBIT_ESC_ROTOR_ALIGN_SUBROUTINE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/subroutines/interface.hpp>

namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Subroutine for aligning the rotor to a known position
   */
  class RotorAlign : public ISubroutine
  {
  public:
    RotorAlign();
    ~RotorAlign();

    void     initialize() final override;
    void     start() final override;
    void     stop() final override;
    void     destroy() final override;
    void     process() final override;
    RunState state() final override;

    size_t mISRTicks;
    bool   mComplete;
  };

}    // namespace Orbit::Control::Subroutine

#endif /* !ORBIT_ESC_ROTOR_ALIGN_SUBROUTINE_HPP */
