/******************************************************************************
 *  File Name:
 *    current_control.cpp
 *
 *  Description:
 *    Implements the current control loop for a FOC motor
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/control/hardware/current_control.hpp>
#include <src/control/foc_data.hpp>
#include <src/control/foc_math.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>

namespace Orbit::Control::Field
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float MAX_DUTY = 0.10f;   /* Testing: Limit max commandable duty cycle? */

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ControlState
  {
    volatile Mode ctl_mode; /**< Current control mode */
    volatile float open_loop_theta;
    volatile float open_loop_iq_ref;
    volatile float open_loop_id_ref;
  };


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile ControlState               s_state;   /**< Current state of the control loop */
  static volatile Chimera::GPIO::Driver_rPtr s_dbg_pin; /**< Debug pin for timing measurements */

  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  static void isr_current_control_loop();

  static void reset_state()
  {
    s_state.ctl_mode = Mode::DISABLED;
    s_state.open_loop_theta = 0.0f;
    s_state.open_loop_iq_ref = 0.0f;
    s_state.open_loop_id_ref = 0.0f;

    foc_ireg_state.iqPID.resetState();
    foc_ireg_state.idPID.resetState();
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize the control state
    -------------------------------------------------------------------------*/
    s_state.ctl_mode = Mode::UNKNOWN;

    /*-------------------------------------------------------------------------
    Get a reference to the debug pin. This is used for timing measurements.
    Works in concert with orbit_motor_sense.cpp.
    -------------------------------------------------------------------------*/
    s_dbg_pin = Chimera::GPIO::getDriver( Orbit::IO::Digital::dbg1Port, Orbit::IO::Digital::dbg1Pin );

    /*-------------------------------------------------------------------------
    Map the current control function to the ADC DMA ISR
    -------------------------------------------------------------------------*/
    Orbit::Motor::Sense::onComplete( isr_current_control_loop );

    /*-------------------------------------------------------------------------
    Initialize the motor drive and feedback sense hardware
    -------------------------------------------------------------------------*/
    Orbit::Motor::Drive::initialize();    // Drive timer first since it's the master
    Orbit::Motor::Sense::initialize();    // Sense timer second since it's the slave

    /*-------------------------------------------------------------------------
    Prepare the system for FOC operation
    -------------------------------------------------------------------------*/
    Orbit::Control::initFOCData();

    /*-------------------------------------------------------------------------
    Assign PID current control parameters
    -------------------------------------------------------------------------*/
    foc_ireg_state.dt = 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ;

    foc_ireg_state.iqPID.init();
    foc_ireg_state.iqPID.OutMinLimit = -20.0f;
    foc_ireg_state.iqPID.OutMaxLimit = 20.0f;
    // foc_ireg_state.iqPID.setTunings( Data::SysControl.currentCtrl_Q_Kp, Data::SysControl.currentCtrl_Q_Ki,
    //                                  Data::SysControl.currentCtrl_Q_Kd, foc_ireg_state.dt );
    foc_ireg_state.iqPID.setTunings( 5.0f, 0.1f, 0.0f, foc_ireg_state.dt );

    foc_ireg_state.idPID.init();
    foc_ireg_state.idPID.OutMinLimit = -20.0f;
    foc_ireg_state.idPID.OutMaxLimit = 20.0f;
    // foc_ireg_state.idPID.setTunings( Data::SysControl.currentCtrl_D_Kp, Data::SysControl.currentCtrl_D_Ki,
    //                                  Data::SysControl.currentCtrl_D_Kd, foc_ireg_state.dt );

    foc_ireg_state.idPID.setTunings( 5.0f, 0.1f, 0.0f, foc_ireg_state.dt );


    foc_ireg_state.mod_vd = 0.0f;
    foc_ireg_state.mod_vq = 0.0f;

    setControlMode( Mode::DISABLED );
  }


  void powerDn()
  {
    /*-------------------------------------------------------------------------
    Tear down in the opposite order of initialization
    -------------------------------------------------------------------------*/
    setControlMode( Mode::DISABLED );
    Orbit::Motor::Sense::reset();
    Orbit::Motor::Drive::reset();

    /*-------------------------------------------------------------------------
    Reset module memory
    -------------------------------------------------------------------------*/
    reset_state();
  }


  bool setControlMode( const Mode mode )
  {
    /*-------------------------------------------------------------------------
    Check if the mode is already set
    -------------------------------------------------------------------------*/
    if( mode == s_state.ctl_mode )
    {
      return true;
    }

    /*-------------------------------------------------------------------------
    Gate the ISR from running temporarily while state data gets updated
    -------------------------------------------------------------------------*/
    auto isr_msk = Chimera::System::disableInterrupts();
    s_state.ctl_mode = Mode::DISABLED;
    Chimera::System::enableInterrupts( isr_msk );

    /*-------------------------------------------------------------------------
    Otherwise, cleanly transition to the new mode
    -------------------------------------------------------------------------*/
    foc_ireg_state.iqPID.resetState();
    foc_ireg_state.idPID.resetState();

    switch( mode )
    {
      case Mode::DISABLED:
        Orbit::Motor::Drive::disableOutput();
        break;

      case Mode::OPEN_LOOP:
        s_state.open_loop_id_ref = 0.0f;
        s_state.open_loop_iq_ref = 0.0f;
        s_state.open_loop_theta  = 0.0f;

        Orbit::Motor::Drive::svmUpdate( 0.0f, 0.0f, 0.0f );
        Orbit::Motor::Drive::enableOutput();
        break;

      case Mode::CLOSED_LOOP:
        foc_motor_state.thetaEst = 0.0f;
        foc_ireg_state.iqRef     = 0.0f;
        foc_ireg_state.idRef     = 0.0f;
        break;

      default:
        Orbit::Motor::Drive::disableOutput();
        return false;
    }

    /*-------------------------------------------------------------------------
    Activate the new mode
    -------------------------------------------------------------------------*/
    s_state.ctl_mode = mode;
    return true;
  }


  Mode getControlMode()
  {
    return s_state.ctl_mode;
  }


  void setInnerLoopReferences( const float iq_ref, const float id_ref, const float theta )
  {
    if( s_state.ctl_mode == Mode::OPEN_LOOP )
    {
      s_state.open_loop_iq_ref = iq_ref;
      s_state.open_loop_id_ref = id_ref;
      s_state.open_loop_theta  = theta;
    }
  }

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Executes a single cycle of the current control algorithm.
   * @note  This function is called from the ADC DMA ISR.
   * @see AN1078: Sensorless Field Oriented Control of PMSM Motors Figure 6
   *
   * This function consumes the latest ADC samples, runs the control algorithm,
   * and updates the PWM outputs for the next cycle.
   */
  static void isr_current_control_loop()
  {
    using namespace Orbit::Motor::Drive;
    using namespace Orbit::Motor::Sense;
    using namespace Orbit::Instrumentation;
    using namespace Orbit::Control::Math;

    /*-------------------------------------------------------------------------
    Decide how to proceed depending on our current mode
    -------------------------------------------------------------------------*/
    float theta = 0.0f;
    float iqRef = 0.0f;
    float idRef = 0.0f;

    switch( s_state.ctl_mode )
    {
      /*-----------------------------------------------------------------------
      Use external references for the current control loop. This is used for
      manual control.
      -----------------------------------------------------------------------*/
      case Mode::OPEN_LOOP:
        theta = s_state.open_loop_theta;
        iqRef = s_state.open_loop_iq_ref;
        idRef = s_state.open_loop_id_ref;
        break;

      /*-----------------------------------------------------------------------
      Use internal references generated by state estimator and speed control
      outer loop to generate current control references.
      -----------------------------------------------------------------------*/
      case Mode::CLOSED_LOOP:
        theta = foc_motor_state.thetaEst;
        iqRef = foc_ireg_state.iqRef;
        idRef = foc_ireg_state.idRef;
        break;

      /*-----------------------------------------------------------------------
      Default case is to do nothing and immediately return.
      -----------------------------------------------------------------------*/
      case Mode::DISABLED:
      default:
        return;
    }

    /*-------------------------------------------------------------------------
    Pull the latest ADC samples
    -------------------------------------------------------------------------*/
    volatile const SenseData &sense_data = getSenseData();
    const float               vSupply    = getSupplyVoltage();

    foc_ireg_state.vma = sense_data.channel[ CHANNEL_PHASE_A_VOLTAGE ];
    foc_ireg_state.vmb = sense_data.channel[ CHANNEL_PHASE_B_VOLTAGE ];
    foc_ireg_state.vmc = sense_data.channel[ CHANNEL_PHASE_C_VOLTAGE ];

    foc_ireg_state.ima = sense_data.channel[ CHANNEL_PHASE_A_CURRENT ];
    foc_ireg_state.imb = sense_data.channel[ CHANNEL_PHASE_B_CURRENT ];
    foc_ireg_state.imc = sense_data.channel[ CHANNEL_PHASE_C_CURRENT ];

    /*-------------------------------------------------------------------------
    Reconstruct 3-phase currents from two phases. One phase could have the
    low side switch active for a very short amount of time, leading to a bad
    ADC sample. By construction, the other two phases will always have ample
    time to sample the current and will be used to reconstruct the third phase
    using Kirchoff's current law.

    See: TIDUCY7 Figure 3. "Using Three-Shunt Current Sampling Technique"
    -------------------------------------------------------------------------*/
    const auto svmState = Orbit::Motor::Drive::getDriver()->svmState();

    if( ( svmState.phase1 == Chimera::Timer::Channel::CHANNEL_2 ) && ( svmState.phase2 == Chimera::Timer::Channel::CHANNEL_3 ) )
    {
      /*-----------------------------------------------------------------------
      Phase A low side is on for the shortest amount of time. Reconstruct it.
      -----------------------------------------------------------------------*/
      foc_ireg_state.imb = sense_data.channel[ CHANNEL_PHASE_B_CURRENT ];
      foc_ireg_state.imc = sense_data.channel[ CHANNEL_PHASE_C_CURRENT ];
      foc_ireg_state.ima = -1.0f * ( foc_ireg_state.imb + foc_ireg_state.imc );
    }
    else if ( ( svmState.phase1 == Chimera::Timer::Channel::CHANNEL_1 ) && ( svmState.phase2 == Chimera::Timer::Channel::CHANNEL_3 ) )
    {
      /*-----------------------------------------------------------------------
      Phase B low side is on for the shortest amount of time. Reconstruct it.
      -----------------------------------------------------------------------*/
      foc_ireg_state.ima = sense_data.channel[ CHANNEL_PHASE_A_CURRENT ];
      foc_ireg_state.imc = sense_data.channel[ CHANNEL_PHASE_C_CURRENT ];
      foc_ireg_state.imb = -1.0f * ( foc_ireg_state.ima + foc_ireg_state.imc );
    }
    else if ( ( svmState.phase1 == Chimera::Timer::Channel::CHANNEL_1 ) && ( svmState.phase2 == Chimera::Timer::Channel::CHANNEL_2 ) )
    {
      /*-----------------------------------------------------------------------
      Phase C low side is on for the shortest amount of time. Reconstruct it.
      -----------------------------------------------------------------------*/
      foc_ireg_state.ima = sense_data.channel[ CHANNEL_PHASE_A_CURRENT ];
      foc_ireg_state.imb = sense_data.channel[ CHANNEL_PHASE_B_CURRENT ];
      foc_ireg_state.imc = -1.0f * ( foc_ireg_state.ima + foc_ireg_state.imb );
    }
    else
    {
      /*-----------------------------------------------------------------------
      Something went wrong, so bugger out now.
      -----------------------------------------------------------------------*/
      emergencyStop();
      RT_DBG_ASSERT( false );
      return;
    }

    /*-------------------------------------------------------------------------
    Use Clarke Transform to convert phase currents from 3-axis to 2-axis
    -------------------------------------------------------------------------*/
    clarke_transform( foc_ireg_state.ima, foc_ireg_state.imb, foc_ireg_state.ia, foc_ireg_state.ib );

    /*-------------------------------------------------------------------------
    Use Park Transform to convert phase currents from 2-axis to d-q axis
    -------------------------------------------------------------------------*/
    park_transform( foc_ireg_state.ia, foc_ireg_state.ib, theta, foc_ireg_state.iq, foc_ireg_state.id );

    /*-------------------------------------------------------------------------
    Run PI controllers for motor currents to generate voltage commands
    -------------------------------------------------------------------------*/
    foc_ireg_state.vq = foc_ireg_state.iqPID.run( iqRef - foc_ireg_state.iq );
    foc_ireg_state.vd = foc_ireg_state.idPID.run( idRef - foc_ireg_state.id );

    // TODO: From mcpwm_foc:4299 (Vedder), once I switch into closed loop control I probably
    // TODO: should add decoupling of the d-q currents.

    // Compute the max length of the voltage space vector without overmodulation
    float max_v_mag = ONE_OVER_SQRT3 * MAX_DUTY * vSupply;

    // Scale the voltage commands to fit within the allowable space vector
    saturate_vector_2d( foc_ireg_state.vd, foc_ireg_state.vq, max_v_mag );

    const float v_norm = 1.5f / vSupply;
    foc_ireg_state.mod_vd = foc_ireg_state.vd * v_norm;
    foc_ireg_state.mod_vq = foc_ireg_state.vq * v_norm;
    // TODO: Possibly filter vq for something later? Not sure yet.

    /*-------------------------------------------------------------------------
    Use Inverse Park Transform to convert d-q voltages to 2-axis phase voltages
    -------------------------------------------------------------------------*/
    inverse_park_transform( foc_ireg_state.mod_vq, foc_ireg_state.mod_vd, theta, foc_ireg_state.va, foc_ireg_state.vb );

    /*-------------------------------------------------------------------------
    Apply the SVM updates
    -------------------------------------------------------------------------*/
    svmUpdate( foc_ireg_state.va, foc_ireg_state.vb, theta );
  }
}    // namespace Orbit::Control::Field
