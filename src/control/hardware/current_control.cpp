/******************************************************************************
 *  File Name:
 *    current_control.cpp
 *
 *  Description:
 *    Implements the current control loop for a FOC motor
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/config/orbit_esc_cfg.hpp>
#include <src/control/foc_data.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/hardware/current_control.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/simulator/sim_adc.hpp>
#include <src/simulator/sim_motor.hpp>


#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* EMBEDDED */

namespace Orbit::Control::Field
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static volatile Mode                        s_ctl_mode;      /**< Current control mode */
  static volatile Chimera::GPIO::Driver_rPtr  s_dbg_pin;       /**< Debug pin for timing measurements */
  static volatile ISRInnerLoopCallback        s_inner_loop_cb; /**< Callback for inner loop custom behaviors */


  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  static void isr_current_control_loop();

  static void reset_state()
  {
    s_ctl_mode = Mode::DISABLED;
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
    s_ctl_mode = Mode::UNKNOWN;
    s_inner_loop_cb = nullptr;

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
    foc_ireg_state.dt = 1.0f / Data::SysControl.statorPWMFreq;

    foc_ireg_state.iqPID.init();
    foc_ireg_state.iqPID.OutMinLimit = -20.0f;
    foc_ireg_state.iqPID.OutMaxLimit = 20.0f;
    // foc_ireg_state.iqPID.setTunings( Data::SysControl.currentCtrl_Q_Kp, Data::SysControl.currentCtrl_Q_Ki,
    //                                  Data::SysControl.currentCtrl_Q_Kd, foc_ireg_state.dt );
    foc_ireg_state.iqPID.setTunings( 5.0f, 0.0f, 0.0f, foc_ireg_state.dt );

    foc_ireg_state.idPID.init();
    foc_ireg_state.idPID.OutMinLimit = -20.0f;
    foc_ireg_state.idPID.OutMaxLimit = 20.0f;
    // foc_ireg_state.idPID.setTunings( Data::SysControl.currentCtrl_D_Kp, Data::SysControl.currentCtrl_D_Ki,
    //                                  Data::SysControl.currentCtrl_D_Kd, foc_ireg_state.dt );
    foc_ireg_state.idPID.setTunings( 5.0f, 0.0f, 0.0f, foc_ireg_state.dt );

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
    if( mode == s_ctl_mode )
    {
      return true;
    }

    Chimera::Timer::Inverter::Driver *const inverter = Motor::Drive::getDriver();

    /*-------------------------------------------------------------------------
    Gate the ISR from running temporarily while state data gets updated
    -------------------------------------------------------------------------*/
    // TODO BMB: This is a bad idea. I need to rework the control loop to perform the algorithm
    // TODO BMB: changes/updates inside the ISR. Enabling/disabling globally is fine though.
    // TODO BMB: I can't allow the motor control signals to be stale for any amount of time. The
    // TODO BMB: update needs to be atomic as far as the power stage is concerned.
    auto isr_msk = Chimera::System::disableInterrupts();
    s_ctl_mode = Mode::DISABLED;
    Chimera::System::enableInterrupts( isr_msk );

    /*-------------------------------------------------------------------------
    Otherwise, cleanly transition to the new mode
    -------------------------------------------------------------------------*/
    foc_ireg_state.iqPID.resetState();
    foc_ireg_state.idPID.resetState();

    switch( mode )
    {
      case Mode::DISABLED:
        inverter->disableOutput();
        #if defined( SIMULATOR )
        Orbit::Sim::ADC::enableMotorSenseADC( false );
        #endif
        break;

      case Mode::OPEN_LOOP:
        foc_motor_state.thetaEst = 0.0f;
        foc_ireg_state.iqRef     = 0.0f;
        foc_ireg_state.idRef     = 0.0f;

        inverter->svmUpdate( 0.0f, 0.0f, 0.0f, 0.0f );
        inverter->enableOutput();

        #if defined( SIMULATOR )
        Orbit::Sim::ADC::enableMotorSenseADC( true );
        #endif
        break;

      case Mode::CLOSED_LOOP:
        // TODO BMB: Honestly this transition needs to happen inside the ISR.
        foc_motor_state.thetaEst = 0.0f;
        foc_ireg_state.iqRef     = 0.0f;
        foc_ireg_state.idRef     = 0.0f;
        break;

      default:
        s_ctl_mode = Mode::DISABLED;
        inverter->disableOutput();

        #if defined( SIMULATOR )
        Orbit::Sim::ADC::enableMotorSenseADC( false );
        #endif
        return false;
    }

    /*-------------------------------------------------------------------------
    Activate the new mode
    -------------------------------------------------------------------------*/
    s_ctl_mode = mode;
    return true;
  }


  Mode getControlMode()
  {
    return s_ctl_mode;
  }


  void setInnerLoopCallback( ISRInnerLoopCallback callback )
  {
    auto isr_msk = Chimera::System::disableInterrupts();
    s_inner_loop_cb = callback;
    Chimera::System::enableInterrupts( isr_msk );
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

    static uint32_t isr_monitor_count = 0;

    Chimera::Timer::Inverter::Driver *const inverter = Motor::Drive::getDriver();

    /*-------------------------------------------------------------------------
    Decide how to proceed depending on our current mode
    -------------------------------------------------------------------------*/
    if( ( s_ctl_mode == Mode::DISABLED ) || !s_inner_loop_cb )
    {
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

    #if defined( EMBEDDED )
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
    #endif  /* EMBEDDED */

    /*-------------------------------------------------------------------------
    Use Clarke Transform to convert phase currents from 3-axis to 2-axis, then
    use Park Transform to convert from 2-axis into the d-q axis.
    -------------------------------------------------------------------------*/
    clarke_transform( foc_ireg_state.ima, foc_ireg_state.imb, foc_ireg_state.ia, foc_ireg_state.ib );

    // TODO BMB: I think I need to update the rotor observer before doing the park transform.
    park_transform( foc_ireg_state.ia, foc_ireg_state.ib, foc_motor_state.thetaEst, foc_ireg_state.iq, foc_ireg_state.id );

    /*-------------------------------------------------------------------------
    Run PI controllers for motor currents to generate voltage commands
    -------------------------------------------------------------------------*/
    foc_ireg_state.vq = foc_ireg_state.iqPID.run( foc_ireg_state.iqRef - foc_ireg_state.iq );
    foc_ireg_state.vd = foc_ireg_state.idPID.run( foc_ireg_state.idRef - foc_ireg_state.id );

    // TODO: From mcpwm_foc:4299 (Vedder), once I switch into closed loop control I probably
    // TODO: should add decoupling of the d-q currents.

    // Compute the max length of the voltage space vector without overmodulation
    float max_v_mag = ONE_OVER_SQRT3 * foc_ireg_state.max_drive * vSupply;

    // Scale the voltage commands to fit within the allowable space vector
    saturate_vector_2d( foc_ireg_state.vd, foc_ireg_state.vq, max_v_mag );

    const float v_norm = 1.5f / vSupply;
    foc_ireg_state.mod_vd = foc_ireg_state.vd * v_norm;
    foc_ireg_state.mod_vq = foc_ireg_state.vq * v_norm;
    // TODO: Possibly filter vq for something later? Not sure yet.

    /*-------------------------------------------------------------------------
    Use Inverse Park Transform to convert d-q voltages to alpha-beta axis, then
    apply the SVM updates.
    -------------------------------------------------------------------------*/
    inverse_park_transform( foc_ireg_state.mod_vq, foc_ireg_state.mod_vd, foc_motor_state.thetaEst, foc_ireg_state.va, foc_ireg_state.vb );

    float modulation_index = hypotf( foc_ireg_state.va, foc_ireg_state.vb );

    inverter->svmUpdate( foc_ireg_state.va, foc_ireg_state.vb, foc_motor_state.thetaEst, modulation_index );

    /*-------------------------------------------------------------------------
    Apply the voltage commands to the simulated motor
    -------------------------------------------------------------------------*/
    #if defined( SIMULATOR )
    static float sim_iq = 0.1f;
    static float sim_id = 0.0f;
    foc_ireg_state.mod_vq = sim_iq;
    foc_ireg_state.mod_vd = sim_id;

    auto motor_state = Orbit::Sim::Motor::modelState();

    inverse_park_transform( foc_ireg_state.mod_vq, foc_ireg_state.mod_vd, motor_state.phi, foc_ireg_state.va, foc_ireg_state.vb );


    Orbit::Sim::Motor::stepModel( foc_ireg_state.va, foc_ireg_state.vb );
    #endif

    /*-------------------------------------------------------------------------
    Invoke control system callback to swap in custom behaviors
    -------------------------------------------------------------------------*/
    s_inner_loop_cb();

    /*-------------------------------------------------------------------------
    Send the control state over the serial port for monitoring
    -------------------------------------------------------------------------*/
    // TODO: if some control flag is set (parameter or compile time)

    #if defined( EMBEDDED )
    if( isr_monitor_count++ >= 15 )
    {
      isr_monitor_count = 0;
    #endif

      /*-----------------------------------------------------------------------
      Pack the message data
      -----------------------------------------------------------------------*/
      Serial::Message::SystemData s_ctl_monitor;

      s_ctl_monitor.raw.header.msgId = MsgId_MSG_SYS_DATA;
      s_ctl_monitor.raw.header.subId = 0;
      s_ctl_monitor.raw.header.uuid  = Serial::Message::getNextUUID();
      s_ctl_monitor.raw.id           = SystemDataId_CURRENT_CONTROL_MONITOR;
      s_ctl_monitor.raw.timestamp    = Chimera::micros();
      s_ctl_monitor.raw.has_payload  = true;
      s_ctl_monitor.raw.payload.size = sizeof( CurrentControlMonitorPayload );

      /*-----------------------------------------------------------------------
      Pack and encode the payload data
      -----------------------------------------------------------------------*/
      Serial::Message::Payload::CurrentControlMonitorPayload payload;

      payload.raw.ia     = foc_ireg_state.ima;
      payload.raw.ib     = foc_ireg_state.imb;
      payload.raw.ic     = foc_ireg_state.imc;
      payload.raw.iq_ref = foc_ireg_state.iqRef;
      payload.raw.id_ref = foc_ireg_state.idRef;
      payload.raw.iq     = foc_ireg_state.iq;
      payload.raw.id     = foc_ireg_state.id;
      payload.raw.vd     = foc_ireg_state.mod_vd;
      payload.raw.vq     = foc_ireg_state.mod_vq;
      payload.raw.va     = foc_ireg_state.va;
      payload.raw.vb     = foc_ireg_state.vb;

      const bool payload_encoded = Serial::Message::encode( &payload.state, Serial::Message::ENCODE_NO_COBS );
      memcpy( s_ctl_monitor.raw.payload.bytes, payload.data(), payload.size() );
      s_ctl_monitor.raw.payload.size = payload.size();

      /*-----------------------------------------------------------------------
      Encode the full message with COBS and queue it for sending. Use best
      effort to send the message, but don't block the control loop.
      -----------------------------------------------------------------------*/
      if( payload_encoded && Serial::Message::encode( &s_ctl_monitor.state ) )
      {
        Serial::getUSBSerialDriver()->writeFromISR( s_ctl_monitor.data(), s_ctl_monitor.size() );
      }
    #if defined( EMBEDDED )
    }
    #endif
  }
}    // namespace Orbit::Control::Field
