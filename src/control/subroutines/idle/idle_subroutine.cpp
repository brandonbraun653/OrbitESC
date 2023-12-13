/******************************************************************************
 *  File Name:
 *    idle_subroutine.cpp
 *
 *  Description:
 *    Basic idling subroutine
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/subroutines/idle/idle_subroutine.hpp>

namespace Orbit::Control::Subroutine
{
  /*-----------------------------------------------------------------------------
  Public Functions
  -----------------------------------------------------------------------------*/
  IdleSubroutine::IdleSubroutine()
  {
    id     = Routine::IDLE;
    mState = State::UNINITIALIZED;
  }

  IdleSubroutine::~IdleSubroutine()
  {
  }


  void IdleSubroutine::initialize()
  {
  }


  void IdleSubroutine::start()
  {
  }


  void IdleSubroutine::stop()
  {
  }


  void IdleSubroutine::destroy()
  {
  }


  void IdleSubroutine::process()
  {
  }


  State IdleSubroutine::state()
  {
    return State::UNINITIALIZED;
  }

}    // namespace Orbit::Control::Subroutine
