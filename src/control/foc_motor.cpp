/******************************************************************************
 *  File Name:
 *    foc_motor.cpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Motor Control. This module is responsible
 *    for implementing the realtime control loops required for FOC. It does not
 *    handle mode transitions or other high level control logic.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/control/foc_motor.hpp>
#include <src/core/hw/orbit_motor.hpp>

namespace Orbit::Control::Field
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ControlState
  {
    Mode ctl_mode; /**< Current control mode */
  };


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile ControlState s_state; /**< Current state of the control loop */
  static volatile Chimera::GPIO::Driver_rPtr s_dbg_pin; /**< Debug pin for timing measurements */

  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Executes a single cycle of the current control algorithm.
   * @note  This function is called from the ADC DMA ISR.
   *
   * This function consumes the latest ADC samples, runs the control algorithm,
   * and updates the PWM outputs for the next cycle.
   */
  static void isr_current_control_loop();

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize the control state
    -------------------------------------------------------------------------*/
    s_state.ctl_mode = Mode::DISABLED;

    /*-------------------------------------------------------------------------
    Get a reference to the debug pin. This is used for timing measurements.
    Works in concert with orbit_motor_sense.cpp.
    -------------------------------------------------------------------------*/
    s_dbg_pin = Chimera::GPIO::getDriver( Orbit::IO::Digital::dbg1Port, Orbit::IO::Digital::dbg1Pin );

    /*-------------------------------------------------------------------------
    Map the current control function to the ADC DMA ISR
    -------------------------------------------------------------------------*/
    Orbit::Motor::setSenseCallback( isr_current_control_loop );

    /*-------------------------------------------------------------------------
    Initialize the motor drive and feedback sense hardware
    -------------------------------------------------------------------------*/
    Orbit::Motor::powerUpDrive(); // Drive timer first since it's the master
    Orbit::Motor::powerUpSense(); // Sense timer second since it's the slave


    // TODO: Move this elsewhere
    Orbit::Motor::enableDriveOutput();
    Orbit::Motor::setDrivePhaseWidth( 40, 50, 60 );
    Orbit::Motor::setDriveCommutation( Orbit::Motor::Rotation::ROTATION_CW, Orbit::Motor::DriveSector::SECTOR_5 );
  }


  bool setControlMode( const Mode mode )
  {
    // TODO
    return false;
  }


  Mode getControlMode()
  {
    return s_state.ctl_mode;
  }

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void isr_current_control_loop()
  {
    using namespace Orbit::Motor;

    volatile const SenseData& sense_data = getSenseData();

    // using namespace Orbit::Control::Math;
    // using namespace Chimera::Timer::Inverter;

    // // static etl::circular_buffer<float, 128> measurement;

    // CurrentControlState *pICtl = &s_state.iLoop;

    // /*-------------------------------------------------------------------------
    // Compute the phase currents. We can only trust two of these as the PWM
    // width for one of the phases might be too small to allow enough settling
    // time for the ADC to get a good reading. Use Kirchoff's current law to get
    // the third phase current.
    // -------------------------------------------------------------------------*/
    // float ia = 0.0f; // Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ CHANNEL_PHASE_A_CURRENT ].measured );
    // float ib = 0.0f; // Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ CHANNEL_PHASE_B_CURRENT ].measured );
    // float ic = 0.0f; // Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ CHANNEL_PHASE_C_CURRENT ].measured );

    // if ( ( pICtl->tOnA >= pICtl->tOnB ) && ( pICtl->tOnA >= pICtl->tOnC ) )
    // {
    //   pICtl->imb = ib;
    //   pICtl->imc = ic;
    //   pICtl->ima = -1.0f * ( pICtl->imb + pICtl->imc );
    // }
    // else if ( ( pICtl->tOnB >= pICtl->tOnA ) && ( pICtl->tOnB >= pICtl->tOnC ) )
    // {
    //   pICtl->ima = ia;
    //   pICtl->imc = ic;
    //   pICtl->imb = -1.0f * ( pICtl->ima + pICtl->imc );
    // }
    // else if ( ( pICtl->tOnC >= pICtl->tOnA ) && ( pICtl->tOnC >= pICtl->tOnB ) )
    // {
    //   pICtl->ima = ia;
    //   pICtl->imb = ib;
    //   pICtl->imc = -1.0f * ( pICtl->ima + pICtl->imb );
    // }
    // else
    // {
    //   pICtl->ima = 0.0f;
    //   pICtl->imb = 0.0f;
    //   pICtl->imc = 0.0f;
    // }

    // pICtl->vm = Orbit::Instrumentation::getSupplyVoltage();

    // // measurement.push( pICtl->imb );

    // /*-------------------------------------------------------------------------
    // Use Clarke Transform to convert phase currents from 3-axis to 2-axis
    // -------------------------------------------------------------------------*/
    // clarke_transform( pICtl->ima, pICtl->imb, pICtl->ia, pICtl->ib );

    // // observerUpdate( s_state );

    // /*-------------------------------------------------------------------------
    // Use Park Transform to convert phase currents from 2-axis to d-q axis
    // -------------------------------------------------------------------------*/
    // park_transform( pICtl->ia, pICtl->ib, pICtl->theta, pICtl->iq, pICtl->id );

    // /*-------------------------------------------------------------------------
    // Use sliding mode controller to estimate rotor speed and position
    // -------------------------------------------------------------------------*/
    // // updateCurrentObserver();
    // // updateSpeedPosObserver();

    // // TODO BMB: Do I need to do something to the output of the clark/park transform to normalize it???
    // // TODO BMB: Why is the calculated iq/id values so much larger than 1?

    // // TODO BMB: Maybe just try plotting alpha/beta and d/q values to see if they are reasonable

    // // if( s_state.switchToClosedLoop )
    // // {
    // /*-------------------------------------------------------------------------
    // Run PI controllers for motor currents to generate voltage commands
    // -------------------------------------------------------------------------*/
    // pICtl->vq -= pICtl->iqPID.run( pICtl->iqRef - pICtl->iq );
    // pICtl->vd += pICtl->idPID.run( pICtl->idRef - pICtl->id );

    // const float max_duty  = 0.20f;
    // const float max_v_mag = ONE_OVER_SQRT3 * max_duty * pICtl->vm;

    // Control::Math::truncate_fabs( pICtl->vd, max_v_mag );

    // float max_vq = sqrtf( ( max_v_mag * max_v_mag ) - ( pICtl->vd * pICtl->vd ) );
    // Control::Math::truncate_fabs( pICtl->vq, max_vq );
    // Control::Math::saturate_vector_2d( pICtl->vd, pICtl->vq, max_v_mag );

    // // ---------
    // const float voltage_normalize = 1.5f / pICtl->vm;
    // const float mod_d             = voltage_normalize * pICtl->vd;
    // const float mod_q             = voltage_normalize * pICtl->vq;

    // /*-------------------------------------------------------------------------
    // Use Inverse Park Transform to convert d-q voltages to 2-axis phase voltages
    // -------------------------------------------------------------------------*/
    // inverse_park_transform( pICtl->vq, pICtl->vd, pICtl->theta, pICtl->va, pICtl->vb );
    // // inverse_park_transform( mod_q, mod_d, pICtl->theta, pICtl->va, pICtl->vb );

    // //   uint32_t sector;
    // //   Control::Math::space_vector_modulation( pICtl->va, pICtl->vb, s_motor_ctrl_timer.getAutoReloadValue(),
    // //                                           pICtl->tOnA,
    // //                                           pICtl->tOnB,
    // //                                           pICtl->tOnC, sector );

    // //   pICtl->activeSector = static_cast<CommutationState>( sector - 1 );
    // //   s_motor_ctrl_timer.setForwardCommState( pICtl->activeSector );
    // //   s_motor_ctrl_timer.setPhaseDutyCycle( pICtl->tOnA, pICtl->tOnB, pICtl->tOnC );
    // // }
    // // else
    // // {
    // /*-------------------------------------------------------------------------
    // Use Inverse Clarke Transform to convert 2-axis phase voltages to 3-axis
    // -------------------------------------------------------------------------*/
    // float pa = 0.0f;
    // float pb = 0.0f;
    // float pc = 0.0f;
    // inverse_clarke_transform( pICtl->va, pICtl->vb, pa, pb, pc );

    // /*-------------------------------------------------------------------------
    // Use SVM to convert 3-axis phase voltages to PWM duty cycles
    // -------------------------------------------------------------------------*/
    // // TODO: Eventually use svm. Currently try out only simple pwm.
    // pICtl->pa = Control::Math::clamp( ( 0.5f * pa ) + 0.5f, 0.0f, 0.20f );
    // pICtl->pb = Control::Math::clamp( ( 0.5f * pb ) + 0.5f, 0.0f, 0.20f );
    // pICtl->pc = Control::Math::clamp( ( 0.5f * pc ) + 0.5f, 0.0f, 0.20f );

    // const uint32_t angle  = static_cast<uint32_t>( RAD_TO_DEG( pICtl->theta ) );
    // pICtl->activeSector   = angle / 60;


    // static etl::circular_buffer<int, 128> commutation_history;
    // commutation_history.push( pICtl->activeSector );

    // /*-------------------------------------------------------------------------
    // Apply the current motor control commands. At its most basic form, the
    // commutation state (and it's rate of change) controls the rotation of the
    // magnetic field vector that actually drives the motor. The phase duty cycle
    // controls the magnitude of the current flowing through the motor coils, which
    // in turn controls the torque and how well the rotor is able to track the
    // magnetic vector.
    // -------------------------------------------------------------------------*/
    // s_motor_ctrl_timer.setForwardCommState( pICtl->activeSector );
    // s_motor_ctrl_timer.setPhaseDutyCycle( pICtl->pa, pICtl->pb, pICtl->pc );

    // uint32_t arr = s_motor_ctrl_timer.getAutoReloadValue();
    // pICtl->tOnA  = static_cast<uint32_t>( pICtl->pa * static_cast<float>( arr ) );
    // pICtl->tOnB  = static_cast<uint32_t>( pICtl->pb * static_cast<float>( arr ) );
    // pICtl->tOnC  = static_cast<uint32_t>( pICtl->pc * static_cast<float>( arr ) );
    // // }

    /*-------------------------------------------------------------------------
    Set the debug pin low to indicate the end of the control loop
    -------------------------------------------------------------------------*/
    s_dbg_pin->setState( Chimera::GPIO::State::LOW );
  }
}    // namespace Orbit::Control::Field
