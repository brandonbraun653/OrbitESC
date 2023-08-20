/******************************************************************************
 *  File Name:
 *    orbit_motor_sense.cpp
 *
 *  Description:
 *    Hardware driver for the motor sensor subsystem. This module is
 *    responsible for reading the motor phase currents and voltages and calling
 *    a handler for processing the results.
 *
 *  Notes:
 *    The ADC is configured to run in DMA mode and trigger off of a timer that
 *    is synchronized with the motor power stage PWM timer. This ensures that
 *    the ADC samples are taken at the same time as the power stage is active.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_motor.hpp>


namespace Orbit::Motor
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct SenseControlBlock
  {
    float timestamp;                  /**< Time stamp in seconds */
    float adc_vref;                   /**< ADC reference voltage */
    float adc_vres;                   /**< ADC resolution in counts/volt */
    float rawData[ CHANNEL_COUNT ];   /**< Raw measured voltage */
    float calOffset[ CHANNEL_COUNT ]; /**< Calibration offset voltage */
    float calData[ CHANNEL_COUNT ];   /**< Calibrated voltage */
    float siData[ CHANNEL_COUNT ];    /**< Data interpreted as SI units */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile SenseControlBlock s_last_sense_data;
  static Chimera::Function::Opaque  s_sense_callback;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Converts an ADC count to a voltage
   *
   * @param counts  ADC count to convert
   * @return float  Voltage in volts
   */
  static inline float counts_to_voltage( const uint16_t counts )
  {
    return ( static_cast<float>( counts ) * s_last_sense_data.adc_vref ) / s_last_sense_data.adc_vres;
  }


  /**
   * @brief Gets the current time in seconds with microsecond precision
   * @return float  Time in seconds
   */
  static inline float current_time_sec()
  {
    return static_cast<float>( Chimera::micros() ) / 1e6f;
  }


  /**
   * @brief Computes phase current from the ADC voltage reading
   *
   * @param vin     ADC voltage reading
   * @return float  Phase current in amps
   */
  static inline float compute_phase_current( const float vin )
  {
    /*-------------------------------------------------------------------------
    OrbitESC v3 circuit parameters
    -------------------------------------------------------------------------*/
    constexpr float ISHUNT_AMP_GAIN = 10.0f; /**< Configured gain of all amplifiers */
    constexpr float RSHUNT_OHM      = 0.01f; /**< Low-side shunt resistor */

    /*-------------------------------------------------------------------------
    Calculate backwards from the ADC reading to what the real current is. The
    op-amp is configured as an inverting amplifier, so a positive current will
    result in a negative voltage delta. The current is then calculated by
    dividing the voltage by the shunt resistor and the op-amp gain.
    -------------------------------------------------------------------------*/
    const float raw = -1.0f * ( vin / ( ISHUNT_AMP_GAIN * RSHUNT_OHM ) );
    return raw;
  }


  /**
   * @brief Computes phase voltage from the ADC voltage reading
   *
   * @param vin     ADC voltage reading
   * @return float  Phase voltage in volts
   */
  static inline float compute_phase_voltage( const float vin )
  {
    /*-------------------------------------------------------------------------
    OrbitESC v3 circuit parameters
    -------------------------------------------------------------------------*/
    constexpr float PHASE_V_SENSE_R1          = 39'000.0f;
    constexpr float PHASE_V_SENSE_R2          = 5'100.0f;
    constexpr float PHASE_V_DIV_RATIO         = PHASE_V_SENSE_R2 / ( PHASE_V_SENSE_R1 + PHASE_V_SENSE_R2 );
    constexpr float ADC_TO_PHASE_V_CONV_CONST = 1.0f / PHASE_V_DIV_RATIO;

    /*-------------------------------------------------------------------------
    Use voltage divider ratio to convert ADC voltage to phase voltage
    -------------------------------------------------------------------------*/
    return vin * ADC_TO_PHASE_V_CONV_CONST;
  }


  /**
   * @brief Callback to handle results of ADC conversions
   *
   * @param isrData Results of the ADC conversion
   * @return void
   */
  static void isr_on_motor_sense_adc_conversion_complete( const Chimera::ADC::InterruptDetail &isr )
  {
    /*-------------------------------------------------------------------------
    Update the sense data cache
    -------------------------------------------------------------------------*/
    s_last_sense_data.timestamp = current_time_sec();
    s_last_sense_data.adc_vref  = isr.vref;
    s_last_sense_data.adc_vres  = isr.resolution;

    for ( auto i = 0; ( i < isr.num_samples ) && ( i < CHANNEL_COUNT ); i++ )
    {
      s_last_sense_data.rawData[ i ] = counts_to_voltage( isr.samples[ i ] );
      s_last_sense_data.calData[ i ] = s_last_sense_data.rawData[ i ] - s_last_sense_data.calOffset[ i ];
    }

    /*-------------------------------------------------------------------------
    Translate raw measurements into representative SI units
    -------------------------------------------------------------------------*/
    s_last_sense_data.siData[ CHANNEL_PHASE_A_CURRENT ] =
        compute_phase_current( s_last_sense_data.calData[ CHANNEL_PHASE_A_CURRENT ] );
    s_last_sense_data.siData[ CHANNEL_PHASE_B_CURRENT ] =
        compute_phase_current( s_last_sense_data.calData[ CHANNEL_PHASE_B_CURRENT ] );
    s_last_sense_data.siData[ CHANNEL_PHASE_C_CURRENT ] =
        compute_phase_current( s_last_sense_data.calData[ CHANNEL_PHASE_C_CURRENT ] );
    s_last_sense_data.siData[ CHANNEL_PHASE_A_VOLTAGE ] =
        compute_phase_voltage( s_last_sense_data.calData[ CHANNEL_PHASE_A_VOLTAGE ] );
    s_last_sense_data.siData[ CHANNEL_PHASE_B_VOLTAGE ] =
        compute_phase_voltage( s_last_sense_data.calData[ CHANNEL_PHASE_B_VOLTAGE ] );
    s_last_sense_data.siData[ CHANNEL_PHASE_C_VOLTAGE ] =
        compute_phase_voltage( s_last_sense_data.calData[ CHANNEL_PHASE_C_VOLTAGE ] );

    /*-------------------------------------------------------------------------
    Invoke the user callback
    -------------------------------------------------------------------------*/
    if ( s_sense_callback )
    {
      s_sense_callback();
    }
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUpSense()
  {
    /*-------------------------------------------------------------------------
    Reset the state of the module
    -------------------------------------------------------------------------*/
    s_sense_callback            = {};
    s_last_sense_data.timestamp = current_time_sec();

    for ( auto i = 0; i < CHANNEL_COUNT; i++ )
    {
      s_last_sense_data.calData[ i ]   = 0.0f;
      s_last_sense_data.calOffset[ i ] = 0.0f;
      s_last_sense_data.rawData[ i ]   = 0.0f;
      s_last_sense_data.siData[ i ]    = 0.0f;
    }

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this module's ISR handler.
    This ADC should already be pre-configured to listen for timer events.
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<isr_on_motor_sense_adc_conversion_complete>();

    auto adc = Chimera::ADC::getDriver( Orbit::IO::Analog::MotorPeripheral );
    adc->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    /*-------------------------------------------------------------------------
    Configure Timer 8 to trigger ADC conversions at a fixed rate
    -------------------------------------------------------------------------*/
    // Use the same period as the motor drive timer
  }


  void setSenseTriggerOffset( const uint32_t offset )
  {
    // configure timer to trigger ADC at offset
  }


  void setSenseCalOffset( const ChannelSequence channel, const float offset )
  {
    if ( channel < CHANNEL_COUNT )
    {
      s_last_sense_data.calOffset[ channel ] = offset;
    }
  }


  void setSenseCallback( Chimera::Function::Opaque &callback )
  {
    s_sense_callback = callback;
  }


  void getSenseData( SenseData &data )
  {
    data.timestamp                          = s_last_sense_data.timestamp;
    data.channel[ CHANNEL_PHASE_A_CURRENT ] = s_last_sense_data.siData[ CHANNEL_PHASE_A_CURRENT ];
    data.channel[ CHANNEL_PHASE_B_CURRENT ] = s_last_sense_data.siData[ CHANNEL_PHASE_B_CURRENT ];
    data.channel[ CHANNEL_PHASE_C_CURRENT ] = s_last_sense_data.siData[ CHANNEL_PHASE_C_CURRENT ];
    data.channel[ CHANNEL_PHASE_A_VOLTAGE ] = s_last_sense_data.siData[ CHANNEL_PHASE_A_VOLTAGE ];
    data.channel[ CHANNEL_PHASE_B_VOLTAGE ] = s_last_sense_data.siData[ CHANNEL_PHASE_B_VOLTAGE ];
    data.channel[ CHANNEL_PHASE_C_VOLTAGE ] = s_last_sense_data.siData[ CHANNEL_PHASE_C_VOLTAGE ];
  }
}    // namespace Orbit::Motor
