/******************************************************************************
 *  File Name:
 *    orbit_i2c.hpp
 *
 *  Description:
 *    Orbit ESC I2C Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_I2C_HPP
#define ORBIT_ESC_I2C_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/i2c>


namespace Orbit::I2C
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
    /**
   * @brief Powers up the I2C driver subsystem
   */
  void powerUp();

}  // namespace Orbit::I2C

#endif  /* !ORBIT_ESC_I2C_HPP */
