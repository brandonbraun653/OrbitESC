/******************************************************************************
 *  File Name:
 *    idle_subroutine.hpp
 *
 *  Description:
 *    Basic idling subroutine
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SUBROUTINE_IDLE_HPP
#define ORBIT_ESC_SUBROUTINE_IDLE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/subroutines/subroutines.hpp>

namespace Orbit::Control::Subroutine
{
  class IdleSubroutine : public ISubroutine
  {
  public:
    IdleSubroutine();
    ~IdleSubroutine();

    void initialize() final override;
    void start() final override;
    void stop() final override;
    void destroy() final override;
    void process() final override;
    State state() final override;
  };
}  // namespace Orbit::Control::Subroutine

#endif  /* !ORBIT_ESC_SUBROUTINE_IDLE_HPP */
