/******************************************************************************
 *  File Name:
 *    adc_runtime.hpp
 *
 *  Description:
 *    Runtime to process ADC
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ADC_RUNTIME_HPP
#define ORBIT_ADC_RUNTIME_HPP

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
   * @brief Initialize the ADC runtime driver for the project
   */
  void initRuntime();

  /**
   * @brief Handles runtime IO of ADC data. Must be called periodically.
   */
  void processADC();

}    // namespace Orbit::ADC

#endif /* !ORBIT_ESC_ADC_RUNTIME_HPP */
