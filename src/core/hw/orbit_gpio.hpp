/******************************************************************************
 *  File Name:
 *    orbit_gpio.hpp
 *
 *  Description:
 *    Orbit ESC GPIO Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_GPIO_HPP
#define ORBIT_ESC_GPIO_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/gpio>


namespace Orbit::GPIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
    /**
   * @brief Powers up the GPIO driver subsystem
   */
  void powerUp();

}  // namespace Orbit::GPIO

#endif  /* !ORBIT_ESC_GPIO_HPP */
