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


namespace Orbit::ADC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the ADC driver subsystem
   */
  void powerUp();

}  // namespace Orbit::ADC

#endif  /* !ORBIT_ESC_ADC_HPP */
