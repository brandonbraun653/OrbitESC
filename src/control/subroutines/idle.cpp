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
#include <src/control/subroutines/declarations.hpp>

namespace Orbit::Control::Subroutine
{
  /*-----------------------------------------------------------------------------
  IdleSubroutine Implementation
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
    mState = State::INITIALIZED;
  }


  void IdleSubroutine::start()
  {
    mState = State::RUNNING;
  }


  void IdleSubroutine::stop()
  {
    mState = State::STOPPED;
  }


  void IdleSubroutine::destroy()
  {
    mState = State::UNINITIALIZED;
  }


  void IdleSubroutine::process()
  {
  }


  State IdleSubroutine::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
