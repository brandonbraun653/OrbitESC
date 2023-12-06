/******************************************************************************
 *  File Name:
 *    orbit_adc.hpp
 *
 *  Description:
 *    Orbit ESC ADC driver interface
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ADC_HPP
#define ORBIT_ESC_ADC_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>


namespace Orbit::ADC
{
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

}    // namespace Orbit::ADC

#endif /* !ORBIT_ESC_ADC_HPP */
