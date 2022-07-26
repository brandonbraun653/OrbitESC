/******************************************************************************
 *  File Name:
 *    foc_driver.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FOC_CONTROL_HPP
#define ORBIT_ESC_FOC_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/common>
#include <Chimera/adc>
#include <Chimera/timer>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class FOC;

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern FOC FOCDriver;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  /**
   * @brief Function signature for a transfer function to convert ADC values to useful
   *
   * @param[in] vin   Input voltage measured by the ADC
   * @return float    Output value of an arbitrary unit
   */
  using ADCTxfrFunc = float ( * )( float vin );


  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum ADCChannel
  {
    ADC_CH_MOTOR_PHASE_A_CURRENT,
    ADC_CH_MOTOR_PHASE_B_CURRENT,
    ADC_CH_MOTOR_SUPPLY_VOLTAGE,

    ADC_CH_NUM_OPTIONS,
    ADC_CH_INVALID
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct FOCConfig
  {
    Chimera::ADC::Peripheral adcSource; /**< Which ADC peripheral to use */

    /*-------------------------------------------------------------------------
    Constants that directly convert between ADC least significant bit voltages
    and the associated SI units.
    -------------------------------------------------------------------------*/
    ADCTxfrFunc phaseACurrentConv; /**< ADC counts -> Amperes for phase A */
    ADCTxfrFunc phaseBCurrentConv; /**< ADC counts -> Amperes for phase B */
    ADCTxfrFunc supplyVoltageConv; /**< ADC counts -> Volts for power supply */

    void clear()
    {
      adcSource         = Chimera::ADC::Peripheral::UNKNOWN;
      phaseACurrentConv = nullptr;
      phaseBCurrentConv = nullptr;
      supplyVoltageConv = nullptr;
    }
  };


  struct ADCData
  {
    float phaseACurrent; /**< Phase A current in Amps */
    float phaseBCurrent; /**< Phase B current in Amps */
    float supplyVoltage; /**< Supply voltage in Volts */

    void clear()
    {
      phaseACurrent = 0.0f;
      phaseBCurrent = 0.0f;
      supplyVoltage = 0.0f;
    }
  };


  struct InternalState
  {
    ADCData adcData;
    float   adcDCOffsets[ ADC_CH_NUM_OPTIONS ];

    void clear()
    {
      adcData.clear();

      for ( auto i = 0; i < ADC_CH_NUM_OPTIONS; i++ )
      {
        adcDCOffsets[ i ] = 0.0f;
      }
    }
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Brush-less dc motor control library implemented with FOC
   *
   */
  class FOC
  {
  public:
    FOC();
    ~FOC();

    int initialize( const FOCConfig &cfg );

    int setSpeedRef( const float ref );

    /**
     * @brief Gets the last data collected from the ADC
     *
     * @param data  The data to fill with the last ADC data
     */
    void lastADCData( ADCData &data );

  protected:
    void dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr );
    void timer_isr_speed_controller();

  private:
    Chimera::ADC::Driver_rPtr        mADCDriver;
    Chimera::Timer::Inverter::Driver mTimerDriver;

    InternalState mPrvState;
    FOCConfig     mConfig;
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_FOC_CONTROL_HPP */
