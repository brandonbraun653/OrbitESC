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


// ! TESTING
#define RPM_TO_COMMUTATION_PERIOD( rpm ) \
  ( 120.0f / ( static_cast<float>( rpm ) * static_cast<float>( Orbit::Data::DFLT_ROTOR_NUM_POLES ) ) )

#define BASE_RPM 1000.0f
// ! TESTING

namespace Orbit::Motor
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using LpFilter = Control::Math::FIR<float, 15>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float A2 = 55.0f;
  static constexpr float A1 = 100.0f;

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
  struct CurrentLoopData
  {
    float ima;   /**< Current measured at phase A terminal */
    float imb;   /**< Current measured at phase B terminal */
    float imc;   /**< Current measured at phase C terminal */
    float vm;    /**< Voltage measured at the motor supply terminal */
    float iq;    /**< Current output measurement for the q-axis */
    float id;    /**< Current output measurement for the d-axis */
    float vq;    /**< Voltage command for the q-axis */
    float vd;    /**< Voltage command for the d-axis */
    float va;    /**< Voltage (alpha) after inverse-park transform */
    float vb;    /**< Voltage (beta) after inverse-park transform */
    float ia;    /**< Current (alpha) after clarke transform */
    float ib;    /**< Current (beta) after clarke transform */
    float theta; /**< Current estimated rotor angle in radians */
    float omega; /**< Current estimated rotor speed in radians/sec */
    float iqRef; /**< Current reference for the q-axis */
    float idRef; /**< Current reference for the d-axis */
    float pa;    /**< Phase A PWM duty cycle */
    float pb;    /**< Phase B PWM duty cycle */
    float pc;    /**< Phase C PWM duty cycle */

    Control::Math::PID iqPID; /**< Current controller for the q-axis */
    Control::Math::PID idPID; /**< Current controller for the d-axis */

    void clear()
    {
      ima   = 0.0f;
      imb   = 0.0f;
      imc   = 0.0f;
      iq    = 0.0f;
      id    = 0.0f;
      vq    = 0.0f;
      vd    = 0.0f;
      va    = 0.0f;
      vb    = 0.0f;
      ia    = 0.0f;
      ib    = 0.0f;
      theta = 0.0f;
      omega = 0.0f;
      iqRef = 0.0f;
      idRef = 0.0f;
      pa    = 0.0f;
      pb    = 0.0f;
      pc    = 0.0f;
      iqPID.init();
      idPID.init();
    }
  };

  struct State
  {
    bool isrControlActive; /**< Flag to enable/disable the ISR control loop */

    uint8_t      commutationPhase; /**< Commutation cycle to be applied */
    ControlState controlState;     /**< Current state of the motor control loop */

    float currentTime;
    float referenceTime;
    float targetRPM;

    CurrentLoopData iLoop; /**< State information for the inner loop controller */
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class ADCControl
  {
  public:
    struct ChannelData
    {
      float    measured;   /**< Measured ADC value, accounting for offset */
      float    calOffset;  /**< The DC offset of the ADC channel at idle */
      size_t   calSamples; /**< How many samples have been acquired */
      float    calSum;     /**< Total sum of the samples */
      LpFilter lpFilter;   /**< Low pass filter for the ADC channel */

      void updateFilter( const LpFilter &filter )
      {
        lpFilter = filter;
        lpFilter.initialize( 0.0f );
      }

      void clear()
      {
        measured  = 0.0f;
        calOffset = 0.0f;
        lpFilter.initialize( 0.0f );
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
  static Chimera::Timer::Inverter::Driver s_motor_ctrl_timer;      /**< Motor drive timer */
  static Chimera::Timer::Trigger::Master  s_speed_ctrl_timer;      /**< Trigger for the speed control loop */
  static ADCControl                       s_adc_control;           /**< ADC control data */
  static State                            s_state;                 /**< State of the motor controller */


  static volatile uint8_t  lastCommutation = 0;
  static volatile uint32_t counter         = 0;
  static float             adcDt           = 0.0f;

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
    static constexpr uint32_t publish_delta_us = 25 * 1000;  /* 40Hz */

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


  static void adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr )
  {
    using namespace Orbit::Control::Math;
    using namespace Chimera::Timer::Inverter;

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

      s_adc_control.data[ i ].measured = correctedVoltage; // s_adc_control.data[ i ].lpFilter.step( correctedVoltage );
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
    if ( !s_state.isrControlActive || s_adc_control.calibrating )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Translate the raw ADC measurements into meaningful values
    -------------------------------------------------------------------------*/
    s_state.iLoop.ima = Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ ADC_CH_MOTOR_PHASE_A_CURRENT ].measured );
    s_state.iLoop.imb = Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ ADC_CH_MOTOR_PHASE_B_CURRENT ].measured );
    s_state.iLoop.imc = Orbit::ADC::sample2PhaseCurrent( s_adc_control.data[ ADC_CH_MOTOR_PHASE_C_CURRENT ].measured );
    s_state.iLoop.vm  = Orbit::ADC::sample2BusVoltage( s_adc_control.data[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].measured );

    /*-------------------------------------------------------------------------
    Use Clarke Transform to convert phase currents from 3-axis to 2-axis
    -------------------------------------------------------------------------*/
    clarke_transform( s_state.iLoop.ima, s_state.iLoop.imb, s_state.iLoop.ia, s_state.iLoop.ib );

    /*-------------------------------------------------------------------------
    Use Park Transform to convert phase currents from 2-axis to d-q axis
    -------------------------------------------------------------------------*/
    park_transform( s_state.iLoop.ia, s_state.iLoop.ib, s_state.iLoop.theta, s_state.iLoop.iq, s_state.iLoop.id );

    /*-------------------------------------------------------------------------
    Use sliding mode controller to estimate rotor speed and position
    -------------------------------------------------------------------------*/
    // TODO: Placeholder for a function call. Will need to select between manual ramp or closed loop
    // s_state.iLoop.omega = 0.0f;
    // s_state.iLoop.theta = 0.0f;

    /*-------------------------------------------------------------------------
    Run PI controllers for motor currents to generate voltage commands
    -------------------------------------------------------------------------*/
    s_state.iLoop.iqRef = 0.0002f; // Simulink model was using this in startup?
    s_state.iLoop.idRef = 0.0f;

    const float iqError = s_state.iLoop.iqRef - s_state.iLoop.iq;
    s_state.iLoop.vq    = s_state.iLoop.iqPID.run( iqError );

    const float idError = s_state.iLoop.idRef - s_state.iLoop.id;
    s_state.iLoop.vd    = s_state.iLoop.idPID.run( idError );

    /*-------------------------------------------------------------------------
    Use Inverse Park Transform to convert d-q voltages to 2-axis phase voltages
    -------------------------------------------------------------------------*/
    inverse_park_transform( s_state.iLoop.vq, s_state.iLoop.vd, s_state.iLoop.theta, s_state.iLoop.va, s_state.iLoop.vb );

    /*-------------------------------------------------------------------------
    Use Inverse Clarke Transform to convert 2-axis phase voltages to 3-axis
    -------------------------------------------------------------------------*/
    float pa = 0.0f;
    float pb = 0.0f;
    float pc = 0.0f;
    inverse_clarke_transform( s_state.iLoop.va, s_state.iLoop.vb, pa, pb, pc );

    /*-------------------------------------------------------------------------
    Use SVM to convert 3-axis phase voltages to PWM duty cycles
    -------------------------------------------------------------------------*/
    // TODO: Eventually use svm. Currently try out only simple pwm.
    s_state.iLoop.pa = Control::Math::clamp( ( 0.5f * pa ) + 0.5f, 0.0f, 1.0f );
    s_state.iLoop.pb = Control::Math::clamp( ( 0.5f * pb ) + 0.5f, 0.0f, 1.0f );
    s_state.iLoop.pc = Control::Math::clamp( ( 0.5f * pc ) + 0.5f, 0.0f, 1.0f );

    const uint32_t         angle  = static_cast<uint32_t>( RAD_TO_DEG( s_state.iLoop.theta ) );
    const uint32_t         sector = angle / 60;
    const CommutationState comm   = static_cast<CommutationState>( sector );
    // Depending on the rotation direction, some one-way hysteresis may be needed

    /*-------------------------------------------------------------------------
    Apply the current motor control commands. At its most basic form, the
    commutation state (and it's rate of change) controls the rotation of the
    magnetic field vector that actually drives the motor. The phase duty cycle
    controls the magnitude of the current flowing through the motor coils, which
    in turn controls the torque and how well the rotor is able to track the
    magnetic vector.
    -------------------------------------------------------------------------*/
    s_motor_ctrl_timer.setForwardCommState( comm );

    /* Current board mapping is:
        Ch1: Phase C
        Ch2: Phase B
        Ch3: Phase A */
    s_motor_ctrl_timer.setPhaseDutyCycle( s_state.iLoop.pc, s_state.iLoop.pb, s_state.iLoop.pa );

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


  static void timer_isr_speed_controller()
  {
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    /*-------------------------------------------------------------------------
    Gate the behavior of this ISR without stopping the Timer/ADC/DMA hardware
    -------------------------------------------------------------------------*/
    s_speed_ctrl_timer.ackISR();
    if ( !s_state.isrControlActive )
    {
      return;
    }

    // s_state.iLoop.theta = 65.0f * 0.0174533f;

    s_state.iLoop.theta += 0.0174533f;    // 1 degree in radians
    if( s_state.iLoop.theta > 6.283185f )
    {
      s_state.iLoop.theta -= 6.283185f;
    }

    /*-------------------------------------------------------------------------
    Push the latest streaming parameters into the transmission buffer
    -------------------------------------------------------------------------*/
    const uint32_t timestamp = Chimera::micros();
    publishPhaseCurrents( timestamp );
    publishPhaseCommands( timestamp );

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
    s_state.isrControlActive = false;
    s_state.commutationPhase = 0;
    s_state.controlState     = ControlState::IDLE;

    // ! Testing
    s_state.currentTime   = 0.0f;
    s_state.referenceTime = 0.0f;
    s_state.targetRPM     = 0.0f;
    adcDt                 = 1.0f / Data::DFLT_STATOR_PWM_FREQ_HZ;

    s_state.iLoop.clear();

    /*
    PID tunings were taken from:
    https://www.mathworks.com/help/mcb/gs/sensorless-foc-pmsm-smo-fo.html
    */
    s_state.iLoop.iqPID.OutMinLimit = -1.0f;
    s_state.iLoop.iqPID.OutMaxLimit = 1.0f;
    s_state.iLoop.iqPID.setTunings( 1.7011f, 0.15494f, 0.0f, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );

    s_state.iLoop.idPID.OutMinLimit = -1.0f;
    s_state.iLoop.idPID.OutMaxLimit = 1.0f;
    s_state.iLoop.idPID.setTunings( 1.7011f, 0.15494f, 0.0f, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );
    // ! Testing

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this class's ISR handler
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
    pwm_cfg.pwmFrequency        = Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ;

    RT_HARD_ASSERT( Chimera::Status::OK == s_motor_ctrl_timer.init( pwm_cfg ) );

    s_motor_ctrl_timer.setPhaseDutyCycle( 0.0f, 0.0f, 0.0f );
    s_motor_ctrl_timer.enableOutput();

    /*-------------------------------------------------------------------------
    Configure the Speed control outer loop update timer
    -------------------------------------------------------------------------*/
    Chimera::Timer::Trigger::MasterConfig trig_cfg;
    trig_cfg.clear();
    trig_cfg.trigFreq               = Orbit::Data::DFLT_SPEED_CTL_UPDT_FREQ_HZ;
    trig_cfg.isrCallback            = Chimera::Function::Opaque::create<timer_isr_speed_controller>();
    trig_cfg.coreConfig.instance    = Chimera::Timer::Instance::TIMER2;
    trig_cfg.coreConfig.baseFreq    = 100'000.0f;
    trig_cfg.coreConfig.clockSource = Chimera::Clock::Bus::SYSCLK;

    RT_HARD_ASSERT( Chimera::Status::OK == s_speed_ctrl_timer.init( trig_cfg ) );
    s_speed_ctrl_timer.enable();


    /*-------------------------------------------------------------------------
    Calibrate the ADC values
    -------------------------------------------------------------------------*/
    // 3kHz pass band
    // auto filter = LpFilter( { 0.00242455316813491f, -0.0101955888092545f, 0.0232392134289419f, -0.0362508890079906f,
    //                           0.0344456824735822f, 0.00415033435064135f, -0.120704132433873f, 0.602608590995338f,
    //                           0.602608590995338f, -0.120704132433873f, 0.00415033435064135f, 0.0344456824735822f,
    //                           -0.0362508890079906f, 0.0232392134289419f, -0.0101955888092545f, 0.00242455316813491f } );

    // 500 Hz pass band
    auto filter = LpFilter( {
        0.00167453239284774f,
        0.00383760414840798f,
        -0.00400032112472084f,
        -0.0269286130929977f,
        -0.0321454769647852f,
        0.0393806301914063f,
        0.191814740418412f,
        0.326079079031429f,
        0.326079079031429f,
        0.191814740418412f,
        0.0393806301914063f,
        -0.0321454769647852f,
        -0.0269286130929977f,
        -0.00400032112472084f,
        0.00383760414840798f,
        0.0016745323928477f,
    } );

    for ( auto &ch : s_adc_control.data )
    {
      ch.updateFilter( filter );
    }

    s_adc_control.startCalibration();
    adc->startSequence();
  }

}    // namespace Orbit::Motor
