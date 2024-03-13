/******************************************************************************
 *  File Name:
 *    rotor_align.cpp
 *
 *  Description:
 *    Implements the rotor alignment routine
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/control/hardware/current_control.hpp>
#include <src/control/subroutines/rotor_align.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/foc_data.hpp>


namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t ALIGNMENT_TICKS = 1000;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile RotorAlign * sRotorAlign;

  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/
  static void isrParkControl();

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  RotorAlign::RotorAlign()
  {
    id        = Routine::FORCE_ALIGNMENT;
    name      = "FOC Rotor Align";
    mState    = RunState::UNINITIALIZED;
    mISRTicks = 0;
    mComplete = false;

    sRotorAlign = this;
  }


  RotorAlign::~RotorAlign()
  {
  }


  void RotorAlign::initialize()
  {
    LOG_INFO( "Initialized %s", this->name.c_str() );

    mISRTicks = ALIGNMENT_TICKS;
    mComplete = false;

    /*-------------------------------------------------------------------------
    Reset the current control loop
    -------------------------------------------------------------------------*/
    initFOCData();
    Field::powerDn();
    Field::powerUp();

    mState = RunState::INITIALIZED;
  }


  void RotorAlign::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );

    /*-----------------------------------------------------------------------------
    Reinitialize the current control loop to drive an alignment current on the
    d-axis. This will force the rotor to align with the stator field.
    -----------------------------------------------------------------------------*/
    Field::setControlMode( Field::Mode::OPEN_LOOP );

    foc_motor_state.thetaEst = DEG_TO_RAD( 120 );
    foc_ireg_state.max_drive = 0.25f;
    foc_ireg_state.iqRef     = 0.0f;
    foc_ireg_state.idRef     = 1.0f;

    mState = RunState::RUNNING;
    Field::setInnerLoopCallback( isrParkControl );
  }


  void RotorAlign::stop()
  {
    /*-------------------------------------------------------------------------
    Disable the inner control loop
    -------------------------------------------------------------------------*/
    foc_ireg_state.iqRef = 0.0f;
    foc_ireg_state.idRef = 0.0f;

    /*-------------------------------------------------------------------------
    Transition to the stopped state
    -------------------------------------------------------------------------*/
    LOG_INFO( "Stopped %s", this->name.c_str() );
    mState = RunState::STOPPED;
  }


  void RotorAlign::destroy()
  {
    mState = RunState::UNINITIALIZED;
  }


  void RotorAlign::process()
  {
    if( mComplete )
    {
      this->stop();
      LOG_INFO( "Alignment complete" );
    }
  }


  RunState RotorAlign::state()
  {
    return mState;
  }


  /*---------------------------------------------------------------------------
  Static Function Implementations
  ---------------------------------------------------------------------------*/

  static void isrParkControl()
  {
    if( !sRotorAlign->mComplete && sRotorAlign->mISRTicks-- == 0 )
    {
      /*-----------------------------------------------------------------------
      We start out driving at 120 degrees, so the second half of the alignment
      routine will be to drive to 0 degrees. Otherwise, we are done.
      -----------------------------------------------------------------------*/
      if( foc_motor_state.thetaEst != 0 )
      {
        foc_motor_state.thetaEst = 0;
        sRotorAlign->mISRTicks = ALIGNMENT_TICKS;
      }
      else
      {
        sRotorAlign->mComplete = true;
      }
    }
  }

}  // namespace Orbit::Control::Subroutine
