/******************************************************************************
 *  File Name:
 *    subroutines.cpp
 *
 *  Description:
 *    Motor control subroutine driver implementation
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/utility>
#include <Chimera/assert>
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
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Performs a reset routine for a given subroutine.
   *
   * @param r The subroutine to be reset.
   * @return true if the reset routine was successful, false otherwise.
   */
  static bool reset_routine( ISubroutine& r )
  {
    r.stop();
    r.destroy();

    if( r.state() != RunState::UNINITIALIZED )
    {
      LOG_WARN( "Unexpected state after manually terminating [%s]", s_curr_routine->name );
      return false;
    }

    return true;
  }


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
      case RunState::UNINITIALIZED:
        s_curr_routine->initialize();
        break;

      case RunState::INITIALIZED:
        s_curr_routine->start();
        break;

      case RunState::RUNNING:
        s_curr_routine->process();
        break;

      case RunState::PANIC:
        s_curr_routine->stop();
        break;

      case RunState::STOPPED:
        s_curr_routine->destroy();
        s_curr_routine = nullptr;
        break;

      default:
        LOG_ERROR( "Unhandled subroutine state %d from [%s]", EnumValue( s_curr_routine->state() ), s_curr_routine->name );
        s_curr_routine = nullptr;
        break;
    }

    /*---------------------------------------------------------------------------
    If we have no routine, switch to the IDLE routine
    ---------------------------------------------------------------------------*/
    if( !s_curr_routine )
    {
      switchRoutine( Routine::IDLE );
    }
  }


  Routine getActiveSubroutine()
  {
    return s_curr_routine->id;
  }


  const char *getSubroutineName( const Routine routine )
  {
    if ( routine < Routine::NUM_OPTIONS )
    {
      return s_routine_map[ EnumValue( routine ) ]->name.c_str();
    }
    else
    {
      return "Subroutine Not Bound";
    }
  }


  bool switchRoutine( const Routine next )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( next >= Routine::NUM_OPTIONS )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Stop the current routine if present
    -------------------------------------------------------------------------*/
    if( s_curr_routine )
    {
      if( s_curr_routine->id == next )
      {
        return true;
      }
      else if( !reset_routine( *s_curr_routine ) )
      {
        return false;
      }
    }

    /*-------------------------------------------------------------------------
    Swap in the requested routine and ensure it's reset
    -------------------------------------------------------------------------*/
    s_curr_routine = s_routine_map[ EnumValue( next ) ];
    return s_curr_routine && reset_routine( *s_curr_routine );
  }

}    // namespace Orbit::Control::Subroutine
