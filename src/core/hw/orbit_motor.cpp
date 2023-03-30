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
#include <src/config/bsp/board_map.hpp>
#include <src/control/filter.hpp>
#include <src/control/foc_math.hpp>
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_motor.hpp>

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
  using LpFilter = Control::Math::FIR<float, 15>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float A1 = 33.0f;
  static constexpr float A2 = 1222.0f;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class ControlState : uint8_t
  {
    IDLE,
    STARTUP,
    RUNNING,
    FAULT
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct State
  {
    bool isrControlActive; /**< Flag to enable/disable the ISR control loop */

    uint8_t      commutationPhase;   /**< Commutation cycle to be applied */
    float        driveStrength[ 3 ]; /**< PWM duty cycle for each phase */
    ControlState controlState;       /**< Current state of the motor control loop */

    float startTime;
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
  static Chimera::Timer::Inverter::Driver s_motor_ctrl_timer; /**< Motor drive timer */
  static Chimera::Timer::Trigger::Master  s_speed_ctrl_timer; /**< Trigger for the speed control loop */
  static ADCControl                       s_adc_control;      /**< ADC control data */
  static volatile State                   s_state;            /**< State of the motor controller */

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr );
  static void timer_isr_speed_controller();

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Reset module memory
    -------------------------------------------------------------------------*/
    s_state.isrControlActive   = false;
    s_state.commutationPhase   = 0;
    s_state.driveStrength[ 0 ] = 10.0f;
    s_state.driveStrength[ 1 ] = 10.0f;
    s_state.driveStrength[ 2 ] = 10.0f;
    s_state.controlState       = ControlState::IDLE;
    s_state.startTime          = 0.0f;

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
    s_motor_ctrl_timer.disableOutput();

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
    auto filter = LpFilter( { 0.00242455316813491f, -0.0101955888092545f, 0.0232392134289419f, -0.0362508890079906f,
                              0.0344456824735822f, 0.00415033435064135f, -0.120704132433873f, 0.602608590995338f,
                              0.602608590995338f, -0.120704132433873f, 0.00415033435064135f, 0.0344456824735822f,
                              -0.0362508890079906f, 0.0232392134289419f, -0.0101955888092545f, 0.00242455316813491f } );

    // 500 Hz pass band
    // auto filter = LpFilter( {
    //   0.00167453239284774f,
    //   0.00383760414840798f,
    //   -0.00400032112472084f,
    //   -0.0269286130929977f,
    //   -0.0321454769647852f,
    //   0.0393806301914063f,
    //   0.191814740418412f,
    //   0.326079079031429f,
    //   0.326079079031429f,
    //   0.191814740418412f,
    //   0.0393806301914063f,
    //   -0.0321454769647852f,
    //   -0.0269286130929977f,
    //   -0.00400032112472084f,
    //   0.00383760414840798f,
    //   0.0016745323928477f,
    // });

    for ( auto &ch : s_adc_control.data )
    {
      ch.updateFilter( filter );
    }

    s_adc_control.startCalibration();
    adc->startSequence();
  }


  static void adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr )
  {
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    /*-------------------------------------------------------------------------
    Process the raw ADC data
    -------------------------------------------------------------------------*/
    const float counts_to_volts = isr.vref / isr.resolution;    // TODO BMB: This should be pre-calculated

    s_adc_control.sampleTimeUs = Chimera::micros();
    for ( size_t i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
    {
      const float sampledAsVoltage = static_cast<float>( isr.samples[ i ] ) * counts_to_volts;
      const float correctedVoltage = sampledAsVoltage - s_adc_control.data[ i ].calOffset;

      s_adc_control.data[ i ].measured = s_adc_control.data[ i ].lpFilter.step( correctedVoltage );
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
    Apply the current motor control commands. At its most basic form, the
    commutation state (and it's rate of change) controls the rotation of the
    magnetic field vector that actually drives the motor. The phase duty cycle
    controls the magnitude of the current flowing through the motor coils, which
    in turn controls the torque and how well the rotor is able to track the
    magnetic vector.
    -------------------------------------------------------------------------*/
    s_motor_ctrl_timer.setForwardCommState( s_state.commutationPhase );
    s_motor_ctrl_timer.setPhaseDutyCycle( s_state.driveStrength[ 0 ], s_state.driveStrength[ 1 ], s_state.driveStrength[ 2 ] );

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

    /*-------------------------------------------------------------------------
    Ramp the motor controller
    -------------------------------------------------------------------------*/
    switch ( s_state.controlState )
    {
      case ControlState::IDLE:
        s_state.controlState = ControlState::STARTUP;
        s_state.startTime = static_cast<float>( Chimera::micros() );
        break;

      case ControlState::STARTUP: {
        /*---------------------------------------------------------------------
        Ramp calculate the new target position
        ---------------------------------------------------------------------*/
        float t = ( static_cast<float>( Chimera::micros() ) - s_state.startTime ) / 1000000.0f;
        float newFreqRef = ( 0.5f * A2 * t * t ) + ( A1 * t );
        float inputAngle = Control::Math::M_2PI_F * newFreqRef * t;
        float position = 0.0f;

        Control::Math::fast_sin( inputAngle, &position );
        float scaledPosition = 360.0f * ( ( position * 0.5f ) + 0.5f ); /* Scale to 0.0f-360.0f*/

        scaledPosition = Control::Math::clamp( scaledPosition, 0.0f, 360.0f );

        /*---------------------------------------------------------------------
        Update the commutation based on the new position
        ---------------------------------------------------------------------*/


        /*---------------------------------------------------------------------
        If we've reached our time limit, move to the next state
        ---------------------------------------------------------------------*/
        if ( t > 1.0f )
        {
          s_state.controlState = ControlState::RUNNING;
        }
        break;

      case ControlState::RUNNING:

        break;

      case ControlState::FAULT:
      default:
        s_motor_ctrl_timer.setForwardCommState( 0 );
        s_motor_ctrl_timer.setPhaseDutyCycle( 0.0f, 0.0f, 0.0f );
        break;
    }

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


}    // namespace Orbit::Motor
