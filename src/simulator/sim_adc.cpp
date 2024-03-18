/******************************************************************************
 *  File Name:
 *    sim_adc.cpp
 *
 *  Description:
 *    ADC simulation model for the test harness
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>
#include <src/simulator/sim_adc.hpp>

#include <thread>
#include <condition_variable>

namespace Orbit::Sim::ADC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static float s_vref;
  static float s_vres;
  static float s_va;
  static float s_vb;
  static float s_vc;
  static float s_ia;
  static float s_ib;
  static float s_ic;
  static float s_vbus;

  static std::condition_variable cv;
  static std::mutex cv_m;
  static bool run = false;
  static std::unique_ptr<std::thread>           s_motor_sense_thread;

  static std::mutex s_data_lock;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Converts an ADC voltage to a count
   *
   * @param voltage  ADC count to convert
   * @return float  Voltage in volts
   */
  static inline uint16_t voltage_to_counts( const float voltage )
  {
    if( s_vref == 0.0f || s_vres == 0.0f )
    {
      return 0;
    }

    return static_cast<uint16_t>( ( voltage * s_vres ) / s_vref );
  }


  /**
   * @brief Converts a raw supply voltage to ADC counts
   *
   * @param voltage  Raw supply voltage in volts
   * @return uint16_t
   */
  static uint16_t supply_voltage_to_adc_counts( const float voltage )
  {
    /*-------------------------------------------------------------------------
    Resistor Divider
    -------------------------------------------------------------------------*/
    constexpr float R1 = 10000.0f;
    constexpr float R2 = 1500.0f;

    const float adc_voltage = voltage * ( R2 / ( R1 + R2 ) );

    return voltage_to_counts( adc_voltage );
  }


  static uint16_t phase_current_to_adc_counts( const float current )
  {
    /*-------------------------------------------------------------------------
    Current Shunt and Op-Amp
    -------------------------------------------------------------------------*/
    constexpr float shunt_resistance = 0.01f;
    constexpr float op_amp_gain      = 10.0f;
    const float adc_voltage          = op_amp_gain * current * shunt_resistance;

    return voltage_to_counts( adc_voltage );
  }


  static uint16_t phase_voltage_to_adc_counts( const float voltage )
  {
    /*-------------------------------------------------------------------------
    Phase Voltage Divider
    -------------------------------------------------------------------------*/
    constexpr float R1 = 39'000.0f;
    constexpr float R2 = 5'100.0f;

    const float adc_voltage = voltage * ( R2 / ( R1 + R2 ) );

    return voltage_to_counts( adc_voltage );
  }

  static void motor_sense_adc_trigger_thread()
  {
    size_t isr_period_us = static_cast<size_t>( ( 1.0f / Data::SysControl.statorPWMFreq ) * 1e6f );


    while( true )
    {
      std::unique_lock<std::mutex> lk( cv_m );
      if( run )
      {
        triggerMotorSenseADC();
      }
      lk.unlock();

      /*-----------------------------------------------------------------------
      Wait for the next trigger event
      -----------------------------------------------------------------------*/
      Chimera::delayMicroseconds( isr_period_us );
    }
  }

  /*-----------------------------------------------------------------------------
  Public Functions
  -----------------------------------------------------------------------------*/
  void initialize( const float vref, const float vres )
  {
    s_vref = vref;
    s_vres = vres;
    s_va   = 0.0f;
    s_vb   = 0.0f;
    s_vc   = 0.0f;
    s_ia   = 0.0f;
    s_ib   = 0.0f;
    s_ic   = 0.0f;
    s_vbus = 12.0f;

    /*-------------------------------------------------------------------------
    Initialize the motor sense ADC interrupt simulation
    -------------------------------------------------------------------------*/
    s_motor_sense_thread = std::make_unique<std::thread>( motor_sense_adc_trigger_thread );
  }


  void enableMotorSenseADC( const bool enable )
  {
    {
      std::lock_guard<std::mutex> lk( cv_m );
      run = enable;
    }

    cv.notify_all();
  }


  void setPhaseData( const float va, const float vb, const float vc, const float ia, const float ib, const float ic )
  {
    {
      std::lock_guard<std::mutex> lk( s_data_lock );
      s_va = va;
      s_vb = vb;
      s_vc = vc;
      s_ia = ia;
      s_ib = ib;
      s_ic = ic;
    }
  }


  void setDCBusVoltage( const float vbus )
  {
    s_vbus = vbus;
  }


  void triggerMotorSenseADC()
  {
    static uint16_t raw_samples[ Orbit::Motor::Sense::CHANNEL_COUNT ];

    Chimera::ADC::InterruptDetail isr_data;
    isr_data.vref        = s_vref;
    isr_data.resolution  = s_vres;
    isr_data.samples     = raw_samples;
    isr_data.num_samples = Orbit::Motor::Sense::CHANNEL_COUNT;

    {
      std::lock_guard<std::mutex> lk( s_data_lock );
      isr_data.samples[ Orbit::Motor::Sense::CHANNEL_PHASE_A_CURRENT ] = phase_current_to_adc_counts( s_ia );
      isr_data.samples[ Orbit::Motor::Sense::CHANNEL_PHASE_B_CURRENT ] = phase_current_to_adc_counts( s_ib );
      isr_data.samples[ Orbit::Motor::Sense::CHANNEL_PHASE_C_CURRENT ] = phase_current_to_adc_counts( s_ic );
      isr_data.samples[ Orbit::Motor::Sense::CHANNEL_PHASE_A_VOLTAGE ] = phase_voltage_to_adc_counts( s_va );
      isr_data.samples[ Orbit::Motor::Sense::CHANNEL_PHASE_B_VOLTAGE ] = phase_voltage_to_adc_counts( s_vb );
      isr_data.samples[ Orbit::Motor::Sense::CHANNEL_PHASE_C_VOLTAGE ] = phase_voltage_to_adc_counts( s_vc );
    }

    Orbit::Motor::Sense::Private::isr_on_motor_sense_adc_conversion_complete( isr_data );
  }


  void triggerInstrumentationADC()
  {
    static uint16_t raw_samples[ Instrumentation::CHANNEL_COUNT ];

    Chimera::ADC::InterruptDetail isr_data;

    isr_data.vref        = s_vref;
    isr_data.resolution  = s_vres;
    isr_data.samples     = raw_samples;
    isr_data.num_samples = Instrumentation::CHANNEL_COUNT;

    isr_data.samples[ Instrumentation::CHANNEL_TEMP ]    = voltage_to_counts( 3.3f / 2.0f );
    isr_data.samples[ Instrumentation::CHANNEL_VSUPPLY ] = supply_voltage_to_adc_counts( s_vbus );
    isr_data.samples[ Instrumentation::CHANNEL_VMCU ]    = voltage_to_counts( 3.3f / 2.0f );
    isr_data.samples[ Instrumentation::CHANNEL_VREF ]    = voltage_to_counts( 1.65f );

    Orbit::Instrumentation::Private::isr_on_instrumentation_adc_conversion_complete( isr_data );
  }

}    // namespace Orbit::Sim::ADC