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
#include <array>
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
  struct EMFObserver
  {
    float Ts;   /**< Sampling time in seconds */

    float d;   /**< Observer eigenvalue selection */
    float z1;
    float z1_dot;
    float z2;
    float z2_dot;

    void clear()
    {
      z1     = 0.0f;
      z1_dot = 0.0f;
      z2     = 0.0f;
      z2_dot = 0.0f;
    }
  };

  struct MotorParameters
  {
    float Ls;   /**< Stator inductance in Henrys */
    float Rs;   /**< Stator resistance in Ohms */

    void clear()
    {
      Ls = 0.0f;
      Rs = 0.0f;
    }
  };


  struct MotorState
  {
    float posRad; /**< Position in radians */
    float velRad; /**< Velocity in radians/second */
    float accRad; /**< Acceleration in radians/second^2 */

    float Id; /**< Current in Amps */
    float Iq; /**< Current in Amps */

    void clear()
    {
      posRad = 0.0f;
      velRad = 0.0f;
      accRad = 0.0f;
    }
  };

  struct FOCConfig
  {
    Chimera::ADC::Peripheral                    adcSource; /**< Which ADC peripheral to use */
    std::array<ADCTxfrFunc, ADC_CH_NUM_OPTIONS> txfrFuncs; /**< Conversion functions for each ADC channel */

    void clear()
    {
      adcSource = Chimera::ADC::Peripheral::UNKNOWN;
      txfrFuncs.fill( nullptr );
    }
  };


  struct ADCSensorData
  {
    float    measured;     /**< Raw ADC value */
    float    converted;    /**< Converted raw value into meaningful data */
    float    dcOffset;     /**< The DC offset of the ADC channel */
    uint32_t sampleTimeUs; /**< The time in microseconds that the ADC sample was taken */

    void clear()
    {
      measured     = 0.0f;
      converted    = 0.0f;
      dcOffset     = 0.0f;
      sampleTimeUs = 0;
    }
  };

  using ADCSensorBuffer = std::array<ADCSensorData, ADC_CH_NUM_OPTIONS>;


  struct InternalState
  {
    ADCSensorBuffer adcBuffer;
    EMFObserver     emfObserver;
    MotorParameters motorParams;
    MotorState      motorState;

    void clear()
    {
      for ( auto &data : adcBuffer )
      {
        data.clear();
      }

      emfObserver.clear();
      motorParams.clear();
      motorState.clear();
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

    int initialize( const FOCConfig &cfg, const MotorParameters &motorParams );

    int setSpeedRef( const float ref );

    /**
     * @brief Gets the last data collected from the ADC
     *
     * @param data  The data to fill with the last ADC data
     */
    void lastSensorData( ADCSensorBuffer &data );

    /**
     * @brief Gets a view of the internal state of the FOC driver
     *
     * @return const InternalState&
     */
    const InternalState& dbgGetState() const;

  protected:
    void dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr );
    void timer_isr_speed_controller();

  private:
    Chimera::ADC::Driver_rPtr        mADCDriver;
    Chimera::Timer::Inverter::Driver mTimerDriver;
    Chimera::Timer::Trigger::Master  mSpeedCtrlTrigger;

    InternalState mState;
    FOCConfig     mConfig;
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_FOC_CONTROL_HPP */
