/******************************************************************************
 *  File Name:
 *    idle_subroutine.cpp
 *
 *  Description:
 *    Basic idling subroutine
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/subroutines/declarations.hpp>

namespace Orbit::Control::Subroutine
{
  /*-----------------------------------------------------------------------------
  IdleSubroutine Implementation
  -----------------------------------------------------------------------------*/
  IdleSubroutine::IdleSubroutine()
  {
    id     = Routine::IDLE;
    name   = "Idle";
    mState = RunState::UNINITIALIZED;
  }

  IdleSubroutine::~IdleSubroutine()
  {
  }


  void IdleSubroutine::initialize()
  {
    mState = RunState::INITIALIZED;
  }


  void IdleSubroutine::start()
  {
    mState = RunState::RUNNING;
  }


  void IdleSubroutine::stop()
  {
    mState = RunState::STOPPED;
  }


  void IdleSubroutine::destroy()
  {
    mState = RunState::UNINITIALIZED;
  }


  void IdleSubroutine::process()
  {
  }


  RunState IdleSubroutine::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
