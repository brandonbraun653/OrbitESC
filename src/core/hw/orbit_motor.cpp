/******************************************************************************
 *  File Name:
 *    orbit_motor.cpp
 *
 *  Description:
 *    Motor controller HW interface implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/timer>
#include <etl/queue_spsc_atomic.h>
#include <etl/circular_buffer.h>
#include <src/config/bsp/board_map.hpp>
#include <src/control/filter.hpp>
#include <src/control/foc_math.hpp>
#include <src/control/pid.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_interface.pb.h>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_usart.hpp>
#include <src/core/runtime/serial_runtime.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/interface/inc/timer>

#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"
#endif /* SEGGER_SYS_VIEW */
#endif /* EMBEDDED */


namespace Orbit::Motor
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class ControlState : uint8_t
  {
    IDLE,
    ALIGN,
    STARTUP,
    RUNNING,
    FAULT
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Stores state for the inner loop current controller
   *
   * Follows the Microchip AN1078 example from Figure 6
   */
  struct CurrentControlState
  {
    float    dt;              /**< Time delta between current control loop invocation */
    float    ima;             /**< Current measured at phase A terminal */
    float    imb;             /**< Current measured at phase B terminal */
    float    imc;             /**< Current measured at phase C terminal */
    float    vm;              /**< Voltage measured at the motor supply terminal */
    float    iq;              /**< Current output measurement for the q-axis */
    float    id;              /**< Current output measurement for the d-axis */
    float    vq;              /**< Voltage command for the q-axis */
    float    vd;              /**< Voltage command for the d-axis */
    float    va;              /**< Voltage (alpha) after inverse-park transform */
    float    vb;              /**< Voltage (beta) after inverse-park transform */
    float    ia;              /**< Current (alpha) after clarke transform */
    float    ib;              /**< Current (beta) after clarke transform */
    float    theta;           /**< Current estimated rotor angle in radians */
    float    omega;           /**< Current estimated rotor speed in radians/sec */
    float    iqRef;           /**< Current reference for the q-axis */
    float    idRef;           /**< Current reference for the d-axis */
    float    pa;              /**< Phase A PWM duty cycle */
    float    pb;              /**< Phase B PWM duty cycle */
    float    pc;              /**< Phase C PWM duty cycle */
    uint32_t tOnA;            /**< Phase A PWM on time */
    uint32_t tOnB;            /**< Phase B PWM on time */
    uint32_t tOnC;            /**< Phase C PWM on time */
    int      activeSector;    /**< Currently active sector */

    Control::Math::PID iqPID; /**< Current controller for the q-axis */
    Control::Math::PID idPID; /**< Current controller for the d-axis */

    void clear()
    {
      dt           = 0.0f;
      ima          = 0.0f;
      imb          = 0.0f;
      imc          = 0.0f;
      iq           = 0.0f;
      id           = 0.0f;
      vq           = 0.0f;
      vd           = 0.0f;
      va           = 0.0f;
      vb           = 0.0f;
      ia           = 0.0f;
      ib           = 0.0f;
      theta        = 0.0f;
      omega        = 0.0f;
      iqRef        = 0.0f;
      idRef        = 0.0f;
      pa           = 0.0f;
      pb           = 0.0f;
      pc           = 0.0f;
      tOnA         = 0;
      tOnB         = 0;
      tOnC         = 0;
      activeSector = Chimera::Timer::Inverter::CommutationState::STATE_OFF;
      iqPID.init();
      idPID.init();
    }
  };

  /**
   * @brief Stores state for the current observer detailed in AN1078
   *
   * This is a state observer that estimates the current flowing through the
   * motor phases. It is used to estimate the back EMF voltage and correct
   * the output of the current controller.
   *
   * This operates on the motor currents after the Clarke transform.
   */
  struct CurrentObserverState
  {
    struct AlgVars
    {
      float  comp_z;         /* Output correction factor voltage */
      float  err_i;          /* Error between measured and estimated current */
      float  est_e;          /* Estimated back EMF */
      float  est_e_filtered; /* Estimated back EMF after filtering */
      float  est_i;          /* Estimated current */
      float  lpf_alpha;      /* Low pass filter alpha value */
      float *p_act_i;        /* Pointer to actual current measurement */
      float *p_cmd_v;        /* Pointer to actual voltage command */
    };

    float   f_gain;     /* Motor parameter based gain tunings */
    float   g_gain;     /* Motor parameter based gain tunings */
    AlgVars phase[ 2 ]; /* Alpha/Beta current observer state */

    void clear()
    {
      f_gain = 0.0f;
      g_gain = 0.0f;
      memset( phase, 0, sizeof( phase ) );
    }
  };


  struct SpeedPosObserverState
  {
    float acc_theta_delta; /**< Accumulated theta delta */
    float lpf_alpha;       /**< Low pass filter alpha value */
    float omega_est;       /**< Estimated rotor speed */
    float omega_filtered;  /**< Estimated rotor speed (filtered) */
    float theta_comp;      /**< Estimated rotor angle (compensated) */
    float theta_est;       /**< Estimated rotor angle */

    void clear()
    {
      acc_theta_delta = 0.0f;
      lpf_alpha       = 0.0f;
      omega_est       = 0.0f;
      omega_filtered  = 0.0f;
      theta_comp      = 0.0f;
      theta_est       = 0.0f;
    }
  };


  struct SpeedControlState
  {
    void clear()
    {
    }
  };


  struct ObserverState
  {
    float x1;
    float x2;
    float lambda_est; /**< Estimate of flux linkage of motor */
    float i_alpha_last;
    float i_beta_last;

    void clear()
    {
      x1           = 0.0f;
      x2           = 0.0f;
      lambda_est   = 0.0f;
      i_alpha_last = 0.0f;
      i_beta_last  = 0.0f;
    }
  };


  struct MotorState
  {
    float foc_motor_flux_linkage;
    float foc_motor_ld_lq_diff;
    float m_gamma_now;          /**< Observer gain */
    float m_phase_now_observer; /**< Observer phase */

    void clear()
    {
      foc_motor_flux_linkage = 0.015f;    // No clue if these are even close
      foc_motor_ld_lq_diff   = 0.0f;
      m_gamma_now            = 500.0f;    // No clue if this is even close
      m_phase_now_observer   = 0.0f;
    }
  };

  struct MotorControlState
  {
    bool                  isrControlActive;   /**< Flag to enable/disable the ISR control loops */
    bool                  switchToClosedLoop; /**< Flag to switch to closed loop control */
    CurrentControlState   iLoop;              /**< Inner loop current controller state data */
    CurrentObserverState  iObserve;           /**< Current observer state data */
    SpeedPosObserverState sObserve;           /**< Speed/position observer state data */
    SpeedControlState     wControl;           /**< Speed controller state data */

    MotorState    motor;
    ObserverState observer;
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class ADCControl
  {
  public:
    struct ChannelData
    {
      float  measured;   /**< Measured ADC value, accounting for offset */
      float  calOffset;  /**< The DC offset of the ADC channel at idle */
      size_t calSamples; /**< How many samples have been acquired */
      float  calSum;     /**< Total sum of the samples */

      void clear()
      {
        measured  = 0.0f;
        calOffset = 0.0f;
      }
    };

    using ChannelBuffer = etl::array<ChannelData, ADC_CH_NUM_OPTIONS>;

    bool          calibrating;  /**< Flag to enable/disable calibration routines */
    size_t        calStartTime; /**< System time when the calibration started */
    size_t        sampleTimeUs; /**< Last time the data was sampled */
    ChannelBuffer data;

    void startCalibration()
    {
      if ( !calibrating )
      {
        for ( auto &ch : data )
        {
          ch.calSamples = 0;
          ch.calSum     = 0.0f;
        }

        calStartTime = Chimera::millis();
        calibrating  = true;
      }
    }

    void clear()
    {
      calibrating = false;
      for ( auto &ch : data )
      {
        ch.clear();
      }
    }
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Inverter::Driver s_motor_ctrl_timer; /**< Motor drive timer */
  static Chimera::Timer::Trigger::Master  s_speed_ctrl_timer; /**< Trigger for the speed control loop */
  static ADCControl                       s_adc_control;      /**< ADC control data */
  static MotorControlState                s_state;            /**< State of the motor controller */

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Captures the latest phase current sample and enqueues it for transmission
   *
   * @param time_us System time in microseconds
   */
  static void publishPhaseCurrents( const uint32_t time_us )
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = 4 * 1000; /* 250Hz */

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamPhaseCurrents || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_ADCPhaseCurrents msg;
    msg.timestamp = time_us;
    msg.ia        = s_state.iLoop.ima;
    msg.ib        = s_state.iLoop.imb;
    msg.ic        = s_state.iLoop.imc;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_ADC_PHASE_CURRENTS;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( SystemDataMessage_ADCPhaseCurrents );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( SystemDataMessage_ADCPhaseCurrents ) );

    Serial::publishDataMessage( sysDataMsg );
  }

  /**
   * @brief Captures the latest phase pwm commands and enqueues it for transmission
   *
   * @param time_us System time in microseconds
   */
  static void publishPhaseCommands( const uint32_t time_us )
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = 25 * 1000; /* 40Hz */

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamPwmCommands || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_PWMCommands msg;
    msg.timestamp = time_us;
    msg.va        = s_state.iLoop.pa;
    msg.vb        = s_state.iLoop.pb;
    msg.vc        = s_state.iLoop.pc;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_PWM_COMMANDS;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( msg );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( msg ) );

    Serial::publishDataMessage( sysDataMsg );
  }


  /**
   * @brief Captures the latest state estimates and enqueues it for transmission
   *
   * @param time_us System time in microseconds
   */
  static void publishStateEstimates( const uint32_t time_us )
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint32_t publish_delta_us = 4 * 1000; /* 100Hz */

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    static uint32_t last_publish = 0;

    /*-------------------------------------------------------------------------
    Wait until it's time to publish the next message
    -------------------------------------------------------------------------*/
    if ( !Data::SysConfig.streamStateEstimates || ( ( time_us - last_publish ) < publish_delta_us ) )
    {
      return;
    }

    last_publish = time_us;

    /*-------------------------------------------------------------------------
    Populate the message
    -------------------------------------------------------------------------*/
    SystemDataMessage_StateEstimates msg;
    msg.timestamp = time_us;
    msg.omega_est = static_cast<float>( s_state.iLoop.activeSector );
    // msg.omega_est = s_state.iLoop.ia;
    // msg.omega_est = s_state.motor.m_phase_now_observer;
    // msg.theta_est = s_state.sObserve.theta_comp;

    /*-------------------------------------------------------------------------
    Pack the message data and publish it
    -------------------------------------------------------------------------*/
    Serial::Message::SysData sysDataMsg;
    sysDataMsg.reset();
    sysDataMsg.payload.header.msgId = MsgId_MSG_SYS_DATA;
    sysDataMsg.payload.header.subId = 0;
    sysDataMsg.payload.header.uuid  = Serial::Message::getNextUUID();
    sysDataMsg.payload.id           = SystemDataId_STATE_ESTIMATES;
    sysDataMsg.payload.has_data     = true;
    sysDataMsg.payload.data.size    = sizeof( msg );
    memcpy( &sysDataMsg.payload.data.bytes, &msg, sizeof( msg ) );

    Serial::publishDataMessage( sysDataMsg );
  }


  /**
   * @brief Updates the current observer with the latest measurements
   *
   * Follows the decision tree presented in Figure 17 of AN1078
   */
  static inline void updateCurrentObserver()
  {
    /*-------------------------------------------------------------------------
    Pre-compute motor dependent constants
    -------------------------------------------------------------------------*/
    static const float nrl  = -1.0f * ( Data::SysConfig.statorResistance / Data::SysConfig.statorInductance );
    static const float invL = 1.0f / Data::SysConfig.statorInductance;

    /*-------------------------------------------------------------------------
    Update the current observer for each phase
    -------------------------------------------------------------------------*/
    for ( uint32_t idx = 0; idx < ARRAY_COUNT( s_state.iObserve.phase ); idx++ )
    {
      CurrentObserverState::AlgVars *pAlg = &s_state.iObserve.phase[ idx ];

      /*-----------------------------------------------------------------------
      Step the virtual motor model
      -----------------------------------------------------------------------*/
      pAlg->est_i = ( nrl * pAlg->est_i ) + ( invL * ( *pAlg->p_cmd_v - pAlg->est_e - pAlg->comp_z ) );

      /*-----------------------------------------------------------------------
      Compute the error signal
      -----------------------------------------------------------------------*/
      pAlg->err_i = pAlg->est_i - *pAlg->p_act_i;
      pAlg->err_i = Control::Math::clamp( pAlg->err_i, -25.0f, 25.0f );

      /*-----------------------------------------------------------------------
      Update the correction factor
      -----------------------------------------------------------------------*/
      if ( fabs( pAlg->err_i ) < Data::SysControl.currentObserver_MaxError )
      {
        pAlg->comp_z = ( pAlg->err_i * Data::SysControl.currentObserver_KSlide ) / Data::SysControl.currentObserver_MaxError;
      }
      else if ( pAlg->err_i > 0 )
      {
        pAlg->comp_z = Data::SysControl.currentObserver_KSlide;
      }
      else
      {
        pAlg->comp_z = -1.0f * Data::SysControl.currentObserver_KSlide;
      }

      /*-----------------------------------------------------------------------
      Update the estimated back EMF w/First Order LPF:
        y[k] = alpha * x[k] + ( 1 - alpha ) * y[k-1]
      -----------------------------------------------------------------------*/
      pAlg->est_e = pAlg->lpf_alpha * pAlg->comp_z + ( 1.0f - pAlg->lpf_alpha ) * pAlg->est_e;

      /*-----------------------------------------------------------------------
      Filter the back EMF estimate for use in the speed/position observer.
      -----------------------------------------------------------------------*/
      pAlg->est_e_filtered = pAlg->lpf_alpha * pAlg->est_e + ( 1.0f - pAlg->lpf_alpha ) * pAlg->est_e_filtered;
    }
  }


  static inline void updateSpeedPosObserver()
  {
    SpeedPosObserverState *pAlg = &s_state.sObserve;

    /*-------------------------------------------------------------------------
    Compute estimated position using arctan(eAlpha/eBeta)
    -------------------------------------------------------------------------*/
    float theta_prv = pAlg->theta_est;
    pAlg->theta_est = Control::Math::fast_atan2_with_norm( s_state.iObserve.phase[ 0 ].est_e_filtered,
                                                           s_state.iObserve.phase[ 1 ].est_e_filtered );

    /*-------------------------------------------------------------------------
    Accumulate change in position over time
    -------------------------------------------------------------------------*/
    pAlg->acc_theta_delta += ( pAlg->theta_est - theta_prv );
  }


  static inline void observerUpdate( MotorControlState &state )
  {
    float R          = Data::SysConfig.statorResistance;
    float L          = Data::SysConfig.statorInductance;
    float lambda     = state.motor.foc_motor_flux_linkage;
    float ld_lq_diff = state.motor.foc_motor_ld_lq_diff;
    float id         = state.iLoop.id;
    float iq         = state.iLoop.iq;

    /*-------------------------------------------------------------------------
    Adjust inductance for saliency
    -------------------------------------------------------------------------*/
    if ( fabsf( id ) > 0.1 || fabsf( iq ) > 0.1 )
    {
      L = L - ld_lq_diff / 2.0f + ld_lq_diff * SQ( iq ) / ( SQ( id ) + SQ( iq ) );
    }

    /*-------------------------------------------------------------------------
    Pre-compute motor dependent constants
    -------------------------------------------------------------------------*/
    float       L_ia       = L * state.iLoop.ia;
    float       L_ib       = L * state.iLoop.ib;
    const float R_ia       = R * state.iLoop.ia;
    const float R_ib       = R * state.iLoop.ib;
    const float gamma_half = state.motor.m_gamma_now * 0.5;

    /*-------------------------------------------------------------------------
    Run the flux-linkage observer

    This implements equation 8 from:
    https://cas.mines-paristech.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    -------------------------------------------------------------------------*/
    float err = SQ( lambda ) - ( SQ( state.observer.x1 - L_ia ) + SQ( state.observer.x2 - L_ib ) );

    // Forcing this term to stay negative helps convergence according to
    //
    // http://cas.ensmp.fr/Publications/Publications/Papers/ObserverPermanentMagnet.pdf
    // and
    // https://arxiv.org/pdf/1905.00833.pdf
    if ( err > 0.0 )
    {
      err = 0.0;
    }

    float x1_dot = state.iLoop.va - R_ia + gamma_half * ( state.observer.x1 - L_ia ) * err;
    float x2_dot = state.iLoop.vb - R_ib + gamma_half * ( state.observer.x2 - L_ib ) * err;

    state.observer.x1 += x1_dot * state.iLoop.dt;
    state.observer.x2 += x2_dot * state.iLoop.dt;

    /*-------------------------------------------------------------------------
    Update state variables and massage bad numbers
    -------------------------------------------------------------------------*/
    state.observer.i_alpha_last = state.iLoop.ia;
    state.observer.i_beta_last  = state.iLoop.ib;

    Control::Math::clear_if_nan( state.observer.x1 );
    Control::Math::clear_if_nan( state.observer.x2 );

    /*-------------------------------------------------------------------------
    Prevent the magnitude from getting too low, which leads to unstable angles
    -------------------------------------------------------------------------*/
    float mag = NORM2_f( state.observer.x1, state.observer.x2 );
    if ( mag < ( lambda * 0.5f ) )
    {
      state.observer.x1 *= 1.1f;
      state.observer.x2 *= 1.1f;
    }

    /*-------------------------------------------------------------------------
    Update phase angle
    -------------------------------------------------------------------------*/
    float y                          = state.observer.x2 - L_ib;
    float x                          = state.observer.x1 - L_ia;
    state.motor.m_phase_now_observer = Control::Math::fast_atan2_with_norm( y, x );
  }


  static inline void runInnerLoopCurrentControl()
  {
    using namespace Orbit::Control::Math;
    using namespace Chimera::Timer::Inverter;

    // static etl::circular_buffer<float, 128> measurement;

    CurrentControlState *pICtl = &s_state.iLoop;

    /*-------------------------------------------------------------------------
    Compute the phase currents. We can only trust two of these as the PWM
    width for one of the phases might be too small to allow enough settling
    time for the ADC to get a good reading. Use Kirchoff's current law to get
    the third phase current.
    -------------------------------------------------------------------------*/
    float ia = Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ ADC_CH_MOTOR_PHASE_A_CURRENT ].measured );
    float ib = Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ ADC_CH_MOTOR_PHASE_B_CURRENT ].measured );
    float ic = Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ ADC_CH_MOTOR_PHASE_C_CURRENT ].measured );

    if ( ( pICtl->tOnA >= pICtl->tOnB ) && ( pICtl->tOnA >= pICtl->tOnC ) )
    {
      pICtl->imb = ib;
      pICtl->imc = ic;
      pICtl->ima = -1.0f * ( pICtl->imb + pICtl->imc );
    }
    else if ( ( pICtl->tOnB >= pICtl->tOnA ) && ( pICtl->tOnB >= pICtl->tOnC ) )
    {
      pICtl->ima = ia;
      pICtl->imc = ic;
      pICtl->imb = -1.0f * ( pICtl->ima + pICtl->imc );
    }
    else if ( ( pICtl->tOnC >= pICtl->tOnA ) && ( pICtl->tOnC >= pICtl->tOnB ) )
    {
      pICtl->ima = ia;
      pICtl->imb = ib;
      pICtl->imc = -1.0f * ( pICtl->ima + pICtl->imb );
    }
    else
    {
      pICtl->ima = 0.0f;
      pICtl->imb = 0.0f;
      pICtl->imc = 0.0f;
    }

    pICtl->vm = Orbit::ADC::sample2BusVoltage( s_adc_control.data[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].measured );

    // measurement.push( pICtl->imb );

    /*-------------------------------------------------------------------------
    Use Clarke Transform to convert phase currents from 3-axis to 2-axis
    -------------------------------------------------------------------------*/
    clarke_transform( pICtl->ima, pICtl->imb, pICtl->ia, pICtl->ib );

    // observerUpdate( s_state );

    /*-------------------------------------------------------------------------
    Use Park Transform to convert phase currents from 2-axis to d-q axis
    -------------------------------------------------------------------------*/
    park_transform( pICtl->ia, pICtl->ib, pICtl->theta, pICtl->iq, pICtl->id );

    /*-------------------------------------------------------------------------
    Use sliding mode controller to estimate rotor speed and position
    -------------------------------------------------------------------------*/
    // updateCurrentObserver();
    // updateSpeedPosObserver();

    // TODO BMB: Do I need to do something to the output of the clark/park transform to normalize it???
    // TODO BMB: Why is the calculated iq/id values so much larger than 1?

    // TODO BMB: Maybe just try plotting alpha/beta and d/q values to see if they are reasonable

    // if( s_state.switchToClosedLoop )
    // {
    /*-------------------------------------------------------------------------
    Run PI controllers for motor currents to generate voltage commands
    -------------------------------------------------------------------------*/
    pICtl->vq -= pICtl->iqPID.run( pICtl->iqRef - pICtl->iq );
    pICtl->vd += pICtl->idPID.run( pICtl->idRef - pICtl->id );

    const float max_duty  = 0.20f;
    const float max_v_mag = ONE_OVER_SQRT3 * max_duty * pICtl->vm;

    Control::Math::truncate_fabs( pICtl->vd, max_v_mag );

    float max_vq = sqrtf( ( max_v_mag * max_v_mag ) - ( pICtl->vd * pICtl->vd ) );
    Control::Math::truncate_fabs( pICtl->vq, max_vq );
    Control::Math::saturate_vector_2d( pICtl->vd, pICtl->vq, max_v_mag );

    // ---------
    const float voltage_normalize = 1.5f / pICtl->vm;
    const float mod_d             = voltage_normalize * pICtl->vd;
    const float mod_q             = voltage_normalize * pICtl->vq;

    /*-------------------------------------------------------------------------
    Use Inverse Park Transform to convert d-q voltages to 2-axis phase voltages
    -------------------------------------------------------------------------*/
    inverse_park_transform( pICtl->vq, pICtl->vd, pICtl->theta, pICtl->va, pICtl->vb );
    // inverse_park_transform( mod_q, mod_d, pICtl->theta, pICtl->va, pICtl->vb );

    //   uint32_t sector;
    //   Control::Math::space_vector_modulation( pICtl->va, pICtl->vb, s_motor_ctrl_timer.getAutoReloadValue(),
    //                                           pICtl->tOnA,
    //                                           pICtl->tOnB,
    //                                           pICtl->tOnC, sector );

    //   pICtl->activeSector = static_cast<CommutationState>( sector - 1 );
    //   s_motor_ctrl_timer.setForwardCommState( pICtl->activeSector );
    //   s_motor_ctrl_timer.setPhaseDutyCycle( pICtl->tOnA, pICtl->tOnB, pICtl->tOnC );
    // }
    // else
    // {
    /*-------------------------------------------------------------------------
    Use Inverse Clarke Transform to convert 2-axis phase voltages to 3-axis
    -------------------------------------------------------------------------*/
    float pa = 0.0f;
    float pb = 0.0f;
    float pc = 0.0f;
    inverse_clarke_transform( pICtl->va, pICtl->vb, pa, pb, pc );

    /*-------------------------------------------------------------------------
    Use SVM to convert 3-axis phase voltages to PWM duty cycles
    -------------------------------------------------------------------------*/
    // TODO: Eventually use svm. Currently try out only simple pwm.
    pICtl->pa = Control::Math::clamp( ( 0.5f * pa ) + 0.5f, 0.0f, 0.20f );
    pICtl->pb = Control::Math::clamp( ( 0.5f * pb ) + 0.5f, 0.0f, 0.20f );
    pICtl->pc = Control::Math::clamp( ( 0.5f * pc ) + 0.5f, 0.0f, 0.20f );

    const uint32_t angle  = static_cast<uint32_t>( RAD_TO_DEG( pICtl->theta ) );
    pICtl->activeSector   = angle / 60;


    static etl::circular_buffer<int, 128> commutation_history;
    commutation_history.push( pICtl->activeSector );

    /*-------------------------------------------------------------------------
    Apply the current motor control commands. At its most basic form, the
    commutation state (and it's rate of change) controls the rotation of the
    magnetic field vector that actually drives the motor. The phase duty cycle
    controls the magnitude of the current flowing through the motor coils, which
    in turn controls the torque and how well the rotor is able to track the
    magnetic vector.
    -------------------------------------------------------------------------*/
    s_motor_ctrl_timer.setForwardCommState( pICtl->activeSector );
    s_motor_ctrl_timer.setPhaseDutyCycle( pICtl->pa, pICtl->pb, pICtl->pc );

    uint32_t arr = s_motor_ctrl_timer.getAutoReloadValue();
    pICtl->tOnA  = static_cast<uint32_t>( pICtl->pa * static_cast<float>( arr ) );
    pICtl->tOnB  = static_cast<uint32_t>( pICtl->pb * static_cast<float>( arr ) );
    pICtl->tOnC  = static_cast<uint32_t>( pICtl->pc * static_cast<float>( arr ) );
    // }
  }


  static inline void runOuterLoopSpeedControl()
  {
    static constexpr float KSpeed = 1.0f;    // TODO: Tune this/make it configurable

    static const float irp_percalc = ( Data::SysControl.statorPWMFreq / Data::SysControl.speedCtrlUpdateFreq );

    SpeedPosObserverState *pAlg = &s_state.sObserve;

    /*-------------------------------------------------------------------------
    Calculate the unfiltered speed estimate
    -------------------------------------------------------------------------*/
    pAlg->omega_est       = pAlg->acc_theta_delta * KSpeed;
    pAlg->acc_theta_delta = 0.0f;

    /*-------------------------------------------------------------------------
    Generate filtered speed estimate with first order low pass filter
    -------------------------------------------------------------------------*/
    pAlg->omega_filtered = pAlg->lpf_alpha * pAlg->omega_est + ( 1.0f - pAlg->lpf_alpha ) * pAlg->omega_filtered;

    /*-------------------------------------------------------------------------
    Compensate the theta estimate for the filtered speed
    -------------------------------------------------------------------------*/
    pAlg->theta_comp = pAlg->theta_est + pAlg->omega_filtered;

    /*-------------------------------------------------------------------------
    Update the LPF gains based on the current speed
    -------------------------------------------------------------------------*/
    // TODO: Don't do this until we have a stable speed estimate
    // pAlg->lpf_alpha = pAlg->omega_filtered * ( Control::Math::M_PI_F / irp_percalc );
    // s_state.iObserve.phase[ 0 ].lpf_alpha = pAlg->lpf_alpha;
    // s_state.iObserve.phase[ 1 ].lpf_alpha = pAlg->lpf_alpha;
  }


  /**
   * @brief Callback to handle results of ADC conversions
   *
   * @param isr  Results of the ADC conversion
   */
  static void adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr )
  {
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    /*-------------------------------------------------------------------------
    Process the raw ADC data
    -------------------------------------------------------------------------*/
    const float counts_to_volts = isr.vref / isr.resolution;

    s_adc_control.sampleTimeUs = Chimera::micros();
    for ( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      const float sampledAsVoltage = static_cast<float>( isr.samples[ i ] ) * counts_to_volts;
      const float correctedVoltage = sampledAsVoltage - s_adc_control.data[ i ].calOffset;

      s_adc_control.data[ i ].measured = correctedVoltage;
      s_adc_control.data[ i ].calSamples++;

      if ( s_adc_control.calibrating && ( s_adc_control.data[ i ].calSamples >= 100 ) )
      {
        s_adc_control.data[ i ].calSum += s_adc_control.data[ i ].measured;
      }
    }

    /*-------------------------------------------------------------------------
    Allow the calibration sequence to proceed if enabled
    -------------------------------------------------------------------------*/
    if ( s_adc_control.calibrating && ( ( Chimera::millis() - s_adc_control.calStartTime ) > 500 ) )
    {
      s_adc_control.calibrating = false;
      s_state.isrControlActive  = true;

      for ( size_t i = 0; i < ADC_CH_MOTOR_SUPPLY_VOLTAGE; i++ )
      {
        s_adc_control.data[ i ].calOffset = s_adc_control.data[ i ].calSum / ( s_adc_control.data[ i ].calSamples - 100 );
      }
    }

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    if ( s_state.isrControlActive && !s_adc_control.calibrating )
    {
      runInnerLoopCurrentControl();
    }

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }

  /**
   * @brief Callback to process the speed controller within a timer ISR
   */
  static void timer_isr_speed_controller()
  {
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    static const float    deg2rad    = 0.0174533f;
    static volatile float start_time = 0.0f;
    static volatile float dt         = 0.0f;

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    s_speed_ctrl_timer.ackISR();
    if ( !s_state.isrControlActive )
    {
      start_time = Chimera::micros() / 1e6f;
      dt         = 0.0f;
      return;
    }

    // !TESTING
    const float ramp_time = ( Chimera::micros() / 1e6f ) - start_time;
    if ( ramp_time < Data::SysControl.rampCtrlRampTimeSec )
    {
      dt = ramp_time;
    }
    else if ( !s_state.switchToClosedLoop )
    {
      s_state.switchToClosedLoop = true;
      s_state.iLoop.vq           = 0.0f;
      s_state.iLoop.vd           = 0.0f;
      s_state.iLoop.idPID.resetState();
      s_state.iLoop.iqPID.resetState();
    }

    /*-----------------------------------------------------------------------------
    Limit the ramp rate of theta so that it can't cross more than one sector
    -----------------------------------------------------------------------------*/
    float dTheta = ( Data::SysControl.rampCtrlSecondOrderTerm * deg2rad * dt * dt ) +
                   ( Data::SysControl.rampCtrlFirstOrderTerm * deg2rad * dt );

    dTheta = Control::Math::clamp( dTheta, 0.0f, DEG_TO_RAD( 59.9f ) );
    s_state.iLoop.theta += dTheta;

    /*-----------------------------------------------------------------------------
    Limit the ramp rate of theta so that it can't cross more than one sector
    -----------------------------------------------------------------------------*/
    if ( s_state.iLoop.theta > 6.283185f )
    {
      s_state.iLoop.theta -= 6.283185f;
    }

    // TODO: Update these with the speed control PI loops
    s_state.iLoop.iqRef = 0.00002f;    // Simulink model was using this in startup?
    s_state.iLoop.idRef = 0.0f;

    // !TESTING

    // runOuterLoopSpeedControl();

    /*-------------------------------------------------------------------------
    Push the latest streaming parameters into the transmission buffer
    -------------------------------------------------------------------------*/
    const uint32_t timestamp = Chimera::micros();
    publishPhaseCurrents( timestamp );
    publishPhaseCommands( timestamp );
    publishStateEstimates( timestamp );

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Reset module memory
    -------------------------------------------------------------------------*/
    s_state.isrControlActive   = false;
    s_state.switchToClosedLoop = false;
    s_state.iLoop.clear();
    s_state.iObserve.clear();
    s_state.wControl.clear();

    // !TESTING
    s_state.observer.clear();
    s_state.motor.clear();

    s_state.motor.m_gamma_now = 2500.0f;
    // !TESTING

    /*-------------------------------------------------------------------------
    Assign PID current control parameters
    -------------------------------------------------------------------------*/
    s_state.iLoop.dt = 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ;

    s_state.iLoop.iqPID.OutMinLimit = -1.0f;
    s_state.iLoop.iqPID.OutMaxLimit = 1.0f;
    s_state.iLoop.iqPID.setTunings( Data::SysControl.currentCtrl_Q_Kp, Data::SysControl.currentCtrl_Q_Ki,
                                    Data::SysControl.currentCtrl_Q_Kd, s_state.iLoop.dt );

    s_state.iLoop.idPID.OutMinLimit = -1.0f;
    s_state.iLoop.idPID.OutMaxLimit = 1.0f;
    s_state.iLoop.idPID.setTunings( Data::SysControl.currentCtrl_D_Kp, Data::SysControl.currentCtrl_D_Ki,
                                    Data::SysControl.currentCtrl_D_Kd, s_state.iLoop.dt );

    /*-------------------------------------------------------------------------
    Initialize the current observer
    -------------------------------------------------------------------------*/
    float initial_alpha = Control::Math::M_PI_F / ( Data::SysControl.statorPWMFreq / Data::SysControl.speedCtrlUpdateFreq );

    /* Pre-calculate F/G gain terms for the current observer */
    s_state.iObserve.f_gain = 1.0f - s_state.iLoop.dt * ( Data::SysConfig.statorResistance / Data::SysConfig.statorInductance );
    s_state.iObserve.g_gain = s_state.iLoop.dt / Data::SysConfig.statorInductance;

    /* Map the phase current alpha/beta references */
    s_state.iObserve.phase[ 0 ].p_act_i = &s_state.iLoop.ia;
    s_state.iObserve.phase[ 1 ].p_act_i = &s_state.iLoop.ib;

    /* Map the phase voltage alpha/beta commands */
    s_state.iObserve.phase[ 0 ].p_cmd_v = &s_state.iLoop.va;
    s_state.iObserve.phase[ 1 ].p_cmd_v = &s_state.iLoop.vb;

    /* Initialize the lpf gains */
    s_state.iObserve.phase[ 0 ].lpf_alpha = initial_alpha;
    s_state.iObserve.phase[ 1 ].lpf_alpha = initial_alpha;
    s_state.sObserve.lpf_alpha            = initial_alpha;

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this module's ISR handler
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<adcISRTxfrComplete>();

    auto adc = Chimera::ADC::getDriver( Orbit::IO::Analog::peripheral );
    adc->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    /*-------------------------------------------------------------------------
    Configure the Advanced Timer for center-aligned 3-phase PWM
    -------------------------------------------------------------------------*/
    Chimera::Timer::Inverter::DriverConfig pwm_cfg;

    pwm_cfg.clear();
    pwm_cfg.coreCfg.instance    = Chimera::Timer::Instance::TIMER1;
    pwm_cfg.coreCfg.clockSource = Chimera::Clock::Bus::SYSCLK;
    pwm_cfg.coreCfg.baseFreq    = 40'000'000.0f;
    pwm_cfg.coreCfg.tolerance   = 1.0f;
    pwm_cfg.adcPeripheral       = Orbit::IO::Analog::peripheral;
    pwm_cfg.adcTriggerOffsetNs  = 50.0f;
    pwm_cfg.adcTriggerSignal    = Chimera::Timer::Trigger::Signal::TRIG_SIG_5;
    pwm_cfg.breakIOLevel        = Chimera::GPIO::State::LOW;
    pwm_cfg.deadTimeNs          = 250.0f;
    pwm_cfg.pwmFrequency        = Orbit::Data::SysControl.statorPWMFreq;

    RT_HARD_ASSERT( Chimera::Status::OK == s_motor_ctrl_timer.init( pwm_cfg ) );

    s_motor_ctrl_timer.setPhaseDutyCycle( 0.0f, 0.0f, 0.0f );
    s_motor_ctrl_timer.enableOutput();

    /*-------------------------------------------------------------------------
    Configure the Speed control outer loop update timer
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = Orbit::Data::SysControl.speedCtrlUpdateFreq;
    trig_cfg.isrCallback            = Chimera::Function::Opaque::create<timer_isr_speed_controller>();
    trig_cfg.coreConfig.instance    = Chimera::Timer::Instance::TIMER2;
    trig_cfg.coreConfig.baseFreq    = 100'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == s_speed_ctrl_timer.init( trig_cfg ) );
    s_speed_ctrl_timer.enable();

    /*-------------------------------------------------------------------------
    Calibrate the ADC values
    -------------------------------------------------------------------------*/
    s_adc_control.startCalibration();
    adc->startSequence();
  }


  void emergencyStop()
  {
    s_motor_ctrl_timer.emergencyBreak();
  }
}    // namespace Orbit::Motor
