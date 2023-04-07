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

    void clear() volatile
    {
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
    }
  };

  struct State
  {
    bool isrControlActive; /**< Flag to enable/disable the ISR control loop */

    uint8_t      commutationPhase;   /**< Commutation cycle to be applied */
    float        driveStrength[ 3 ]; /**< PWM duty cycle for each phase */
    ControlState controlState;       /**< Current state of the motor control loop */

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
  static Chimera::Timer::Inverter::Driver s_motor_ctrl_timer; /**< Motor drive timer */
  static Chimera::Timer::Trigger::Master  s_speed_ctrl_timer; /**< Trigger for the speed control loop */
  static ADCControl                       s_adc_control;      /**< ADC control data */
  static State                   s_state;            /**< State of the motor controller */


  static volatile uint8_t  lastCommutation = 0;
  static volatile uint32_t counter         = 0;
  static float             adcDt           = 0.0f;

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


    // ! Testing
    s_state.currentTime   = 0.0f;
    s_state.referenceTime = 0.0f;
    s_state.targetRPM     = 0.0f;
    adcDt                 = 1.0f / Data::DFLT_STATOR_PWM_FREQ_HZ;

    s_state.iLoop.clear();
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

    using namespace Orbit::Control::Math;

    /*-------------------------------------------------------------------------
    Use Clarke Transform to convert phase currents from 3-axis to 2-axis
    -------------------------------------------------------------------------*/
    clarke_transform( s_adc_control.data[ ADC_CH_MOTOR_PHASE_A_CURRENT ].measured,
                      s_adc_control.data[ ADC_CH_MOTOR_PHASE_B_CURRENT ].measured, s_state.iLoop.ia, s_state.iLoop.ib );

    /*-------------------------------------------------------------------------
    Use Park Transform to convert phase currents from 2-axis to d-q axis
    -------------------------------------------------------------------------*/
    park_transform( s_state.iLoop.ia, s_state.iLoop.ib, s_state.iLoop.theta, s_state.iLoop.iq, s_state.iLoop.id );

    /*-------------------------------------------------------------------------
    Use sliding mode controller to estimate rotor speed and position
    -------------------------------------------------------------------------*/
    // TODO: Placeholder for a function call. Will need to select between manual ramp or closed loop

    /*-------------------------------------------------------------------------
    Run PI controllers for motor currents to generate voltage commands
    -------------------------------------------------------------------------*/


    /*-------------------------------------------------------------------------
    Use Inverse Park Transform to convert d-q currents to 2-axis phase voltages
    -------------------------------------------------------------------------*/


    /*-------------------------------------------------------------------------
    Use Inverse Clarke Transform to convert 2-axis phase voltages to 3-axis
    -------------------------------------------------------------------------*/


    /*-------------------------------------------------------------------------
    Use SVM to convert 3-axis phase voltages to PWM duty cycles
    -------------------------------------------------------------------------*/


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


#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


}    // namespace Orbit::Motor
