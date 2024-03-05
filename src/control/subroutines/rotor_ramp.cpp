/******************************************************************************
 *  File Name:
 *    rotor_ramp.cpp
 *
 *  Description:
 *    Implementation of the rotor ramping subroutine. This is attempting to
 *    follow the description from the following white paper (Section 3):
 *
 *  https://scolton-www.s3.amazonaws.com/motordrive/sensorless_gen1_Rev1.pdf
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/control/hardware/current_control.hpp>
#include <src/control/subroutines/rotor_ramp.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/foc_data.hpp>
#include <src/core/data/orbit_data.hpp>

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

  struct RampState
  {
    RampStep rampStep;     /**< Current step in the ramping process */
    uint32_t startTimeRef; /**< Time reference for the start of a process */
    float    omega_desired;       /**< Desired final rotor speed */
    uint32_t rampStart_us; /**< When the ramp started in microseconds */
  };

  /*---------------------------------------------------------------------------
  Temporary Values
  ---------------------------------------------------------------------------*/
  static constexpr float s_rpm_desired = 15000.0f;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static RampState mRampState;

  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  static void isrParkControl();
  static void isrRampControl();

  /*---------------------------------------------------------------------------
  RotorRamp Implementation
  ---------------------------------------------------------------------------*/

  RotorRamp::RotorRamp()
  {
    id     = Routine::OPEN_LOOP_RAMP_FOC;
    name   = "FOC Rotor Ramp";
    mState = RunState::UNINITIALIZED;
  }


  RotorRamp::~RotorRamp()
  {
  }


  void RotorRamp::initialize()
  {
    LOG_INFO( "Initialized %s", this->name.c_str() );
    mState = RunState::INITIALIZED;

    initFOCData();

    /*-------------------------------------------------------------------------
    Reset the current control loop
    -------------------------------------------------------------------------*/
    Field::powerDn();
    Field::powerUp();
  }


  void RotorRamp::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );

    /*-----------------------------------------------------------------------------
    Reinitialize the current control loop
    -----------------------------------------------------------------------------*/
    Field::setControlMode( Field::Mode::OPEN_LOOP );

    mRampState.rampStep     = RampStep::ALIGN;
    mRampState.startTimeRef = Chimera::millis();

    foc_motor_state.thetaEst = Data::SysControl.parkTheta;
    foc_ireg_state.iqRef     = 0.1f;
    foc_ireg_state.idRef     = 0.0f;

    mState = RunState::RUNNING;
    Field::setInnerLoopCallback( isrParkControl );
  }


  void RotorRamp::stop()
  {
    LOG_INFO( "Stopped %s", this->name.c_str() );
    mState = RunState::STOPPED;
  }


  void RotorRamp::destroy()
  {
    mState = RunState::UNINITIALIZED;
  }


  void RotorRamp::process()
  {
    switch( mRampState.rampStep )
    {
      // TODO BMB: Make align time and strength a parameter
      /*-----------------------------------------------------------------------
      Perform the alignment control in a low speed open loop fashion. This
      step is used to get the rotor in a known position before ramping up the
      magnetic vector. After a certain amount of time, transition to ramping.
      -----------------------------------------------------------------------*/
      case RampStep::ALIGN:
        if( ( Chimera::millis() - mRampState.startTimeRef ) > 1000 )
        {
          mRampState.startTimeRef = Chimera::millis();
          mRampState.rampStep     = RampStep::RAMP;

          /*-------------------------------------------------------------------
          Engage the high speed current control loop behavior
          -------------------------------------------------------------------*/
          foc_motor_state.thetaEst = Data::SysControl.parkTheta;
          foc_ireg_state.iqRef     = 0.0f;
          foc_ireg_state.idRef     = 0.0f;

          // TODO: I'm going to need to scale this omega by the rotor "gearing" ratio
          mRampState.rampStart_us  = Chimera::micros();
          mRampState.omega_desired = ( s_rpm_desired / 60.0f ) * Math::M_2PI_F;
          foc_ireg_state.max_drive = 0.0f;
          foc_ireg_state.iqRef     = 0.0f;
          Field::setInnerLoopCallback( isrRampControl );
        }
        break;

      /*-----------------------------------------------------------------------
      Ramp control happens inside the high speed current loop. This allows for
      a cycle-by-cycle control of the current vector. This case looks for the
      vector rotation rate to reach the user's idle speed before transitioning
      to the complete state.
      -----------------------------------------------------------------------*/
      case RampStep::RAMP:
        if( foc_motor_state.omegaEst >= mRampState.omega_desired )
        {
          mRampState.rampStep = RampStep::COMPLETE;
        }
        break;

      case RampStep::COMPLETE:

        break;

      case RampStep::ERROR:
      default:

        break;
    }
  }


  RunState RotorRamp::state()
  {
    return mState;
  }


  /*---------------------------------------------------------------------------
  Static Function Implementations
  ---------------------------------------------------------------------------*/

  static void isrParkControl()
  {
    // Don't need to do anything right now. Could get fancy later with more dynamic control.
  }

  /**
   * @brief Controls the ramping of the rotor magnetic vector.
   *
   * Executes in the context of the inner current control loop ISR. This will
   * bring the rotor from a standstill up to the user's desired idle speed.
   */
  static void isrRampControl()
  {
    /*-------------------------------------------------------------------------
    Update the angular rate according to the ramp function
    -------------------------------------------------------------------------*/
    if( foc_motor_state.omegaEst < mRampState.omega_desired )
    {
      const float now_sec     = static_cast<float>( Chimera::micros() - mRampState.rampStart_us ) * 1e-6f;
      const float omega_scale = now_sec / 3.0f;

      foc_motor_state.omegaEst = mRampState.omega_desired * omega_scale;
      foc_ireg_state.max_drive = 0.3f * omega_scale;
      foc_ireg_state.iqRef     = 2.0f;
    }
    else
    {
      foc_ireg_state.iqRef = 0.5f;
    }

    /*-------------------------------------------------------------------------
    Compute the next theta vector for the rotor given the current angular rate
    -------------------------------------------------------------------------*/
    const float dTheta = foc_motor_state.omegaEst / Data::SysControl.statorPWMFreq;

    foc_motor_state.thetaEst += dTheta;
    Math::normalize_radians( foc_motor_state.thetaEst );
  }

}    // namespace Orbit::Control::Subroutine
