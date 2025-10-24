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
#include <src/control/hardware/speed_control.hpp>
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
    RAMP,     /**< Controlled ramp from zero to idle speed */
    COMPLETE, /**< Rotor is at idle speed */
    ERROR     /**< Ramp subroutine failed */
  };

  struct RampState
  {
    RampStep rampStep;      /**< Current step in the ramping process */
    float    omega_desired; /**< Desired final rotor speed */
    uint32_t rampStart_us;  /**< When the ramp started in microseconds */
  };

  /*---------------------------------------------------------------------------
  Temporary Values
  ---------------------------------------------------------------------------*/
  static constexpr float s_rpm_desired = 1000.0f;
  static bool            s_transitioned;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static RampState mRampState;

  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

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

    // TEMPORARY!
    s_transitioned = false;
  }


  void RotorRamp::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );

    /*-----------------------------------------------------------------------------
    Reinitialize the current control loop
    -----------------------------------------------------------------------------*/
    Field::setControlMode( Field::Mode::OPEN_LOOP );

    mRampState.rampStep     = RampStep::RAMP;
    mRampState.rampStart_us = Chimera::micros();

    // TODO BMB: Need to use the actual motor poles here to calculate the desired electrical speed
    // TODO BMB: Need to use a parameter setpoint for the RPM
    mRampState.omega_desired = 7.0f * ( s_rpm_desired / 60.0f ) * Math::M_2PI_F;

    foc_motor_state.thetaEst = 0.0f;
    foc_motor_state.omegaEst = 0.0f;
    foc_ireg_state.iqRef     = 0.0f;
    foc_ireg_state.idRef     = 0.0f;

    mState = RunState::RUNNING;
    Field::setInnerLoopCallback( isrRampControl );
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

  /**
   * @brief Controls the ramping of the rotor magnetic vector.
   *
   * Executes in the context of the inner current control loop ISR. This will
   * bring the rotor from a standstill up to the user's desired idle speed.
   */
  static void isrRampControl()
  {
    using namespace Orbit::Control;

    const size_t curr_time = Chimera::micros();
    const size_t delta_us  = curr_time - mRampState.rampStart_us;
    const float  now_sec   = static_cast<float>( delta_us ) * 1e-6f;

    if( s_transitioned )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Update the angular rate according to the ramp function
    -------------------------------------------------------------------------*/
    if( foc_motor_state.omegaEst < mRampState.omega_desired )
    {
      foc_ireg_state.max_drive = 1.0f;
      foc_ireg_state.iqRef     = 1.0f;
      foc_ireg_state.idRef     = 0.0f;

      // TODO: Needs to be omega_e(k) = min(omega_{exit}, omega_e(k-1) + a*T_s), where T_s is sample time.
      foc_motor_state.omegaEst = mRampState.omega_desired * now_sec;
    }

    if( now_sec > 3.0f )
    {
      /*-----------------------------------------------------------------------
      TESTING: Very naively transition to closed loop control once we've idled
      for a bit. Let's just see what happens.
      -----------------------------------------------------------------------*/
      s_transitioned = true;
      Field::setControlMode( Field::Mode::CLOSED_LOOP );
      Speed::setControlMode( Speed::Mode::CLOSED_LOOP );
    }
  }

}    // namespace Orbit::Control::Subroutine
