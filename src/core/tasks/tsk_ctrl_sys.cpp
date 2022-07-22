/******************************************************************************
 *  File Name:
 *    tsk_sys_ctrl.cpp
 *
 *  Description:
 *    Control system task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_ctrl_sys.hpp>
#include <src/control/foc_driver.hpp>

namespace Orbit::Tasks::CTRLSYS
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Convert the ADC counts to the voltage on the DC bus
   * @note Only used for the BOOTXL-DRV8301
   *
   * DC Bus Voltage:
   *  gain = (vin_real_max/vin_adc_max) * (adc_vref/adc_max_counts)
   *       = (26.314/3.3) * (3.3/4096)
   *       =  7.97 * 0.000805664
   *       =  0.006424316
   *
   * @param[in] counts  The ADC counts to convert
   * @return The voltage on the DC bus
   */
  static float adcCountsToDCBusVoltage( uint16_t counts, float dc_offset )
  {
    static constexpr float DC_BUS_VOLTAGE_CONV_GAIN = 0.006424316f;
    return ( static_cast<float>( counts ) * DC_BUS_VOLTAGE_CONV_GAIN );
  }


  /**
   * @brief Convert the ADC counts to the current on the phase A motor
   * @note Only used for the BOOSTXL-DRV8301
   *
   * adc_vref = 3.3v
   * adc_max_counts = 4096
   * op_amp_gain = 10
   * R = 0.01
   *
   * Phase A/B Current:
   *  IR = (v_adc - 1.65)/op_amp_gain
   *   I = ((v_adc - 1.65)/op_amp_gain)/R
   *     = ((counts * (adc_vref/adc_max_counts)) - 1.65)/op_amp_gain)/R
   *     = <plug in values and reduce in WolframAlpha>
   *     = 0.00805664 * counts - 16.5
   *
   * @param[in] counts  The ADC counts to convert
   * @return The current on the phase A or B winding
   */
  static float adcCountsToPhaseABCurrent( uint16_t counts, float dc_offset )
  {
    return ( 0.00805664f * static_cast<float>( counts ) - 16.5f );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void CTRLSYSThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize the CTRLSYS drivers
    -------------------------------------------------------------------------*/
    Orbit::Control::FOC foc;
    Orbit::Control::FOCConfig cfg;

    cfg.supplyVoltageConv = adcCountsToDCBusVoltage;
    cfg.phaseACurrentConv = adcCountsToPhaseABCurrent;
    cfg.phaseBCurrentConv = adcCountsToPhaseABCurrent;


    cfg.adcSource = Chimera::ADC::Peripheral::ADC_0;

    foc.initialize( cfg );

    /*-------------------------------------------------------------------------
    Run the CTRLSYS thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while ( 1 )
    {
      /*---------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      ---------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}    // namespace Orbit::Tasks::CTRLSYS
