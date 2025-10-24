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
#include <etl/queue_spsc_atomic.h>
#include <src/config/bsp/board_map.hpp>
#include <src/config/orbit_esc_cfg.hpp>
#include <src/control/foc_data.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/foc_observer.hpp>
#include <src/control/hardware/current_control.hpp>
#include <src/control/hardware/speed_control.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_usb.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/trace/orbit_trace.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/simulator/sim_adc.hpp>
#include <src/simulator/sim_observer.hpp>

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* EMBEDDED */

namespace Orbit::Control::Field
{
  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  static void reset_state();

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static volatile Mode                       s_ctl_mode;      /**< Current control mode */
  static volatile Chimera::GPIO::Driver_rPtr s_dbg_pin;       /**< Debug pin for timing measurements */
  static volatile ISRInnerLoopCallback       s_inner_loop_cb; /**< Callback for inner loop custom behaviors */

  static etl::queue_spsc_atomic<uint8_t, 4096, etl::memory_model::MEMORY_MODEL_MEDIUM> s_tx_isr_buffer;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize the control state
    -------------------------------------------------------------------------*/
    s_ctl_mode      = Mode::UNKNOWN;
    s_inner_loop_cb = nullptr;
    s_tx_isr_buffer.clear();

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
    Orbit::Control::Observer::initialize();
    Orbit::Control::Observer::setPolicy( Orbit::Control::Observer::Policy::ORTEGA_NON_LINEAR );

    /*-------------------------------------------------------------------------
    Assign PID current control parameters
    -------------------------------------------------------------------------*/
    foc_ireg_state.dt = 1.0f / Data::SysControl.statorPWMFreq;

    foc_ireg_state.iqPID.init();
    foc_ireg_state.iqPID.OutMinLimit = -12.0f;
    foc_ireg_state.iqPID.OutMaxLimit = 12.0f;
    // foc_ireg_state.iqPID.setTunings( Data::SysControl.currentCtrl_Q_Kp, Data::SysControl.currentCtrl_Q_Ki,
    //                                  Data::SysControl.currentCtrl_Q_Kd, foc_ireg_state.dt );
    foc_ireg_state.iqPID.setTunings( 15.0f, 0.1f, 0.0f, foc_ireg_state.dt );

    foc_ireg_state.idPID.init();
    foc_ireg_state.idPID.OutMinLimit = -12.0f;
    foc_ireg_state.idPID.OutMaxLimit = 12.0f;
    // foc_ireg_state.idPID.setTunings( Data::SysControl.currentCtrl_D_Kp, Data::SysControl.currentCtrl_D_Ki,
    //                                  Data::SysControl.currentCtrl_D_Kd, foc_ireg_state.dt );
    foc_ireg_state.idPID.setTunings( 15.0f, 0.1f, 0.0f, foc_ireg_state.dt );

    foc_ireg_state.vd_mod = 0.0f;
    foc_ireg_state.vq_mod = 0.0f;

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
    s_ctl_mode   = Mode::DISABLED;
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
    auto isr_msk    = Chimera::System::disableInterrupts();
    s_inner_loop_cb = callback;
    Chimera::System::enableInterrupts( isr_msk );
  }


  void pumpISRDataStream()
  {
    using namespace Orbit::Serial;

    /*-------------------------------------------------------------------------
    Send the control state over the serial port for monitoring
    -------------------------------------------------------------------------*/
    if( s_tx_isr_buffer.empty() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Pull the next byte from the buffer and send it over the serial port
    -------------------------------------------------------------------------*/
    auto     serial    = getUSBSerialDriver();
    uint32_t idx       = 0;
    size_t   pump_size = 0;
    uint8_t  arr[ 256 ];

    Chimera::Thread::TimedLockGuard lck( *serial );
    if( !lck.try_lock_for( Chimera::Thread::TIMEOUT_DONT_WAIT ) )
    {
      return;
    }

    while( s_tx_isr_buffer.size() )
    {
      /*-----------------------------------------------------------------------
      Pull a number of bytes from the buffer
      -----------------------------------------------------------------------*/
      idx       = 0;
      pump_size = std::min( sizeof( arr ), serial->availableForWrite() );

      memset( arr, 0, sizeof( arr ) );

      while( s_tx_isr_buffer.pop( arr[ idx ] ) && ( idx < pump_size ) )
      {
        idx++;
      }

      /*-----------------------------------------------------------------------
      Send the data over the serial port. We can't do anything if the data
      can't enqueue, so just drop it.
      -----------------------------------------------------------------------*/
      serial->write( arr, idx );
    }
  }


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  static void reset_state()
  {
    s_ctl_mode = Mode::DISABLED;
    foc_ireg_state.iqPID.resetState();
    foc_ireg_state.idPID.resetState();
  }


  /**
   * @brief Executes a single cycle of the current control algorithm.
   * @note  This function is called from the ADC DMA ISR.
   * @see AN1078: Sensorless Field Oriented Control of PMSM Motors Figure 6
   *
   * This function consumes the latest ADC samples, runs the control algorithm,
   * and updates the PWM outputs for the next cycle.
   */
  void isr_current_control_loop()
  {
    using namespace Orbit::Motor::Drive;
    using namespace Orbit::Motor::Sense;
    using namespace Orbit::Instrumentation;
    using namespace Orbit::Control::Math;

    static uint32_t         isr_monitor_count = 0;
    static Observer::Input  observer_input;
    static Observer::Output observer_output;

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
    else if( ( svmState.phase1 == Chimera::Timer::Channel::CHANNEL_1 ) &&
             ( svmState.phase2 == Chimera::Timer::Channel::CHANNEL_3 ) )
    {
      /*-----------------------------------------------------------------------
      Phase B low side is on for the shortest amount of time. Reconstruct it.
      -----------------------------------------------------------------------*/
      foc_ireg_state.ima = sense_data.channel[ CHANNEL_PHASE_A_CURRENT ];
      foc_ireg_state.imc = sense_data.channel[ CHANNEL_PHASE_C_CURRENT ];
      foc_ireg_state.imb = -1.0f * ( foc_ireg_state.ima + foc_ireg_state.imc );
    }
    else if( ( svmState.phase1 == Chimera::Timer::Channel::CHANNEL_1 ) &&
             ( svmState.phase2 == Chimera::Timer::Channel::CHANNEL_2 ) )
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
#endif /* EMBEDDED */

    /*-------------------------------------------------------------------------
    Use Clarke Transform to convert from rotational frame => alpha/beta frame.
    -------------------------------------------------------------------------*/
    clarke_transform( foc_ireg_state.ima, foc_ireg_state.imb, foc_ireg_state.ialpha, foc_ireg_state.ibeta );
    clarke_transform( foc_ireg_state.vma, foc_ireg_state.vmb, foc_ireg_state.valpha, foc_ireg_state.vbeta );

    /*-------------------------------------------------------------------------
    Run the obvserver to update the system estimation. This will always track
    in open loop, but the values will actually get used once we switch over to
    closed loop speed control.
    -------------------------------------------------------------------------*/
    observer_input.dt     = foc_ireg_state.dt;
    observer_input.iAlpha = foc_ireg_state.ialpha;
    observer_input.iBeta  = foc_ireg_state.ibeta;
    observer_input.vAlpha = foc_ireg_state.va_cmd;
    observer_input.vBeta  = foc_ireg_state.vb_cmd;

#if defined( EMBEDDED )
    Observer::execute( observer_input, observer_output );
#elif defined( SIMULATOR )
    Orbit::Sim::Control::Observer::execute( observer_input, observer_output );
#endif

    if( s_ctl_mode == Mode::CLOSED_LOOP )
    {
      foc_motor_state.thetaEst = observer_output.theta_elec;
      foc_motor_state.omegaEst = observer_output.omega_elec;
    }
    else
    {
      /*-----------------------------------------------------------------------
      Open loop integration of the commanded speed
      -----------------------------------------------------------------------*/
      // Math::normalize_radians( foc_motor_state.thetaEst ); or modulo 2pi? Is that a thing?
      // theta_e(k) = theta_e(k-1) + omega_e(k)*T_s (unwrap to avoid jumps)
    }

    /*-------------------------------------------------------------------------
    Using the new estimations, convert to the DQ axis for control
    -------------------------------------------------------------------------*/
    park_transform( foc_ireg_state.ialpha, foc_ireg_state.ibeta, foc_motor_state.thetaEst, foc_ireg_state.iq,
                    foc_ireg_state.id );

    /*-------------------------------------------------------------------------
    Generate voltage commands in the D-Q axis for the next control cycle. This
    regulates the inner loop currents.
    -------------------------------------------------------------------------*/
    // TODO: From mcpwm_foc:4299 (Vedder), once I switch into closed loop control I probably
    // TODO: should add decoupling of the d-q currents.
    foc_ireg_state.vd = foc_ireg_state.idPID.run( foc_ireg_state.idRef - foc_ireg_state.id );
    foc_ireg_state.vq = foc_ireg_state.iqPID.run( foc_ireg_state.iqRef - foc_ireg_state.iq );

    /*-------------------------------------------------------------------------
    Modulate the voltage commands to fit within the allowable space vector
    -------------------------------------------------------------------------*/
    // Compute the max length of the voltage space vector without overmodulation
    float max_v_mag = ONE_OVER_SQRT3 * foc_ireg_state.max_drive * vSupply;

    // Scale the voltage commands to fit within the allowable space vector
    saturate_vector_2d( foc_ireg_state.vd, foc_ireg_state.vq, max_v_mag );

    const float v_norm    = 1.5f / vSupply;
    foc_ireg_state.vd_mod = foc_ireg_state.vd * v_norm;
    foc_ireg_state.vq_mod = foc_ireg_state.vq * v_norm;

    /*-------------------------------------------------------------------------
    Convert rotating DQ frame back to stationary alpha-beta frame
    -------------------------------------------------------------------------*/
    inverse_park_transform( foc_ireg_state.vq_mod, foc_ireg_state.vd_mod, foc_motor_state.thetaEst, foc_ireg_state.va_cmd,
                            foc_ireg_state.vb_cmd );

    /*-------------------------------------------------------------------------
    Update the SVM to generate the next PWM cycle
    -------------------------------------------------------------------------*/
    float modulation_index = hypotf( foc_ireg_state.va_cmd, foc_ireg_state.vb_cmd );

    // TODO: Grok says I don't need theta here as it can be computed internally from
    // TODO: alpha/beta via atan2(beta, alpha). Might be worth it?
    inverter->svmUpdate( foc_ireg_state.va_cmd, foc_ireg_state.vb_cmd, foc_motor_state.thetaEst, modulation_index );

    /*-------------------------------------------------------------------------
    Invoke control system callback to swap in custom inner loop behaviors
    -------------------------------------------------------------------------*/
    if( s_inner_loop_cb )
    {
      s_inner_loop_cb();
    }
  }
}    // namespace Orbit::Control::Field
