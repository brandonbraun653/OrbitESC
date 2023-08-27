/******************************************************************************
 *  File Name:
 *    orbit_adc.hpp
 *
 *  Description:
 *    Orbit ESC ADC driver interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ADC_HPP
#define ORBIT_ESC_ADC_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <etl/array.h>


namespace Orbit::ADC
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct IPhaseCal
  {
    float dcOffset; /**< Averaged DC offset when inputs are shorted */
    float floor;    /**< Minimum value seen */
    float ceiling;  /**< Maximum value seen */
  };

  using IPhaseCalArray = etl::array< IPhaseCal, 3>;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the ADC driver subsystem
   */
  void powerUp();

  /**
   * @brief Computes the total time it takes to sample all the motor channels in nanoseconds
   * @return uint32_t
   */
  uint32_t motorChannelSampleTimeNs();

  /**
   * @brief Calibrate the motor current sensors
   * @note This function will block for a few seconds
   *
   * @param cal           Calibration data structure
   * @param sampleTimeMs  Time to sample for each channel
   */
  void calibrateCurrentSensors( IPhaseCalArray &cal, const size_t sampleTimeMs );

}  // namespace Orbit::ADC

#endif  /* !ORBIT_ESC_ADC_HPP */
