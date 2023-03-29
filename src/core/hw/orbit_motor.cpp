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
#include <src/core/data/orbit_data_defaults.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/control/filter.hpp>

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
  Structures
  ---------------------------------------------------------------------------*/
  struct State
  {
    bool isrControlActive; /**< Flag to enable/disable the ISR control loop */
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
  static State                            s_state;            /**< State of the motor controller */

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

    // const float phaseACurrent = mConfig.txfrFuncs[ ADC_CH_MOTOR_PHASE_A_CURRENT ]( s_adc_control.data[
    // ADC_CH_MOTOR_PHASE_A_CURRENT ].measured ); const float phaseBCurrent = mConfig.txfrFuncs[ ADC_CH_MOTOR_PHASE_B_CURRENT ](
    // s_adc_control.data[ ADC_CH_MOTOR_PHASE_B_CURRENT ].measured ); const float phaseCCurrent = mConfig.txfrFuncs[
    // ADC_CH_MOTOR_PHASE_C_CURRENT ]( s_adc_control.data[ ADC_CH_MOTOR_PHASE_C_CURRENT ].measured ); const float busVoltage =
    // mConfig.txfrFuncs[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ]( s_adc_control.data[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ].measured );

    // /*-----------------------------------------------------------------------
    // Move the sampled phase currents through the Clarke-Park transform

    // TODO: Use 3-phase current measurements due to slew rate improvement.s
    // -----------------------------------------------------------------------*/
    // const auto clarke = Math::clarke_transform( phaseACurrent, phaseBCurrent );
    // const auto park = Math::park_transform( clarke, mState.motorCtl.posEst );

    // /*-----------------------------------------------------------------------
    // Run the control loop
    // -----------------------------------------------------------------------*/
    // /* Update the motor state data from previous calculations */
    // mState.motorCtl.Iqm = park.q;
    // mState.motorCtl.Idm = park.d;
    // mState.motorCtl.Vdd = busVoltage;

    // /* Low-pass filter the measured currents */
    // mState.motorCtl.Idf = mState.motorCtl.DFIR.step( mState.motorCtl.Idm );
    // mState.motorCtl.Iqf = mState.motorCtl.QFIR.step( mState.motorCtl.Iqm );

    // /* Step the PID controllers */
    // mState.motorCtl.Dpid.run( mState.motorCtl.Idr - mState.motorCtl.Idf );
    // mState.motorCtl.Vdr = mState.motorCtl.Dpid.Output;

    // mState.motorCtl.Qpid.run( mState.motorCtl.Iqr - mState.motorCtl.Iqf );
    // mState.motorCtl.Vqr = mState.motorCtl.Qpid.Output;

    // /* Open loop control motor speed and position estimates */
    // mState.motorCtl.spdEst = mState.motorCtl.IqrInt.step( mState.motorCtl.Iqr, ( 1.0f / Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ
    // ) ); mState.motorCtl.posEst = mState.motorCtl.SpdInt.step( mState.motorCtl.spdEst, ( 1.0f /
    // Orbit::Data::DFLT_STATOR_PWM_FREQ_HZ ) );

    // /* Naive wrapping of position */
    // if( mState.motorCtl.posEst > Math::M_2PI_F )
    // {
    //   mState.motorCtl.posEst -= Math::M_2PI_F;
    // }
    // else if( mState.motorCtl.posEst < 0.0f )
    // {
    //   mState.motorCtl.posEst += Math::M_2PI_F;
    // }

    // /*-------------------------------------------------------------------------
    // Update the commanded controller output states
    // -------------------------------------------------------------------------*/
    // float                 a, b, c;
    // const Math::ParkSpace park_space{ .d = mState.motorCtl.Vdr, .q = mState.motorCtl.Vqr };
    // auto                  invPark = Math::inverse_park_transform( park_space, mState.motorCtl.posEst );

    // Math::inverse_clarke_transform( invPark, &a, &b, &c );

    // /*-------------------------------------------------------------------------
    // Use the position estimate to determine the next commutation state. Split
    // the unit circle into 6 sectors and select the appropriate one to commutate.

    // https://math.stackexchange.com/a/206662/435793
    // Solve for N => Theta * 3 / pi
    // -------------------------------------------------------------------------*/
    // static constexpr float SECTOR_CONV_FACTOR = 3.0f / Math::M_PI_F;
    // uint8_t                sector = 1 + static_cast<uint8_t>( SECTOR_CONV_FACTOR * mState.motorCtl.posEst );

    // // mState.motorCtl.svpwm_a_duty = ( a + 0.5f ) * 100.0f;
    // // mState.motorCtl.svpwm_b_duty = ( b + 0.5f ) * 100.0f;
    // // mState.motorCtl.svpwm_c_duty = ( c + 0.5f ) * 100.0f;
    // // mState.motorCtl.svpwm_comm   = sector;

    // static uint32_t counter = 0;
    // mState.motorCtl.svpwm_a_duty = 10.0f;
    // mState.motorCtl.svpwm_b_duty = 10.0f;
    // mState.motorCtl.svpwm_c_duty = 10.0f;

    // counter++;
    // if( counter > 10 )
    // {
    //   counter = 0;
    //   mState.motorCtl.svpwm_comm++;
    //   if( mState.motorCtl.svpwm_comm >= 7 )
    //   {
    //     mState.motorCtl.svpwm_comm = 1;
    //   }
    // }


    // /* Step the EMF observer */
    // const float dt = US_TO_SEC( timestamp_us - mState.emfObserver.last_update_us );
    // stepEMFObserver( dt );
    // mState.emfObserver.last_update_us = timestamp_us;

    // /* Calculate the estimated rotor position (Eq. 18) */
    // mState.motorController.posEstRad = Math::fast_atan2_with_norm( -mState.emfObserver.Ed_est, mState.emfObserver.Eq_est );
    // if ( mState.motorController.velEstRad < 0.0f )
    // {
    //   mState.motorController.posEstRad += Math::M_PI_F;
    // }

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
    SEGGER_SYSVIEW_RecordExitISR();
#endif
  }


}    // namespace Orbit::Motor
