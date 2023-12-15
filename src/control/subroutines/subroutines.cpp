/******************************************************************************
 *  File Name:
 *    subroutines.cpp
 *
 *  Description:
 *    Motor control subroutine driver implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <Aurora/utility>
#include <etl/array.h>
#include <src/control/subroutines/interface.hpp>

namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using RoutineArray = etl::array<ISubroutine *, EnumValue( Routine::NUM_OPTIONS )>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static RoutineArray  s_routine_map;   /**< Storage of bound routines */
  static ISubroutine * s_curr_routine;  /**< Currently executing routine */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    s_routine_map.fill( nullptr );
  }


  void bind( const Routine routine, ISubroutine *const pimpl )
  {
    if ( routine < Routine::NUM_OPTIONS )
    {
      if( pimpl )
      {
        RT_HARD_ASSERT( pimpl->id == routine );
        pimpl->initialize();
      }

      s_routine_map[ EnumValue( routine ) ] = pimpl;
    }
  }


  void process()
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !s_curr_routine )
    {
      return;
    }

    /*---------------------------------------------------------------------------
    Run the state machine for the current routine
    ---------------------------------------------------------------------------*/
    switch( s_curr_routine->state() )
    {
      case State::UNINITIALIZED:
        s_curr_routine->initialize();
        break;

      case State::INITIALIZED:
        s_curr_routine->start();
        break;

      case State::RUNNING:
        s_curr_routine->process();
        break;

      case State::PANIC:
        s_curr_routine->stop();
        break;

      case State::STOPPED:
        s_curr_routine->destroy();
        break;

      default:
        break;
    }

    /*-------------------------------------------------------------------------
    Process pending requests for state transitions
    -------------------------------------------------------------------------*/

    /*-------------------------------------------------------------------------
    Process pending requests for routine transitions
    -------------------------------------------------------------------------*/
    // maybe do another mini state machine here to handle the transition
  }


  Routine currentRoutine()
  {
    return s_curr_routine->id;
  }


  bool modifyState( const Request request )
  {
    return false;
  }


  bool switchRoutine( const Routine next )
  {
    return false;
  }

}    // namespace Orbit::Control::Subroutine
