/******************************************************************************
 *  File Name:
 *    orbit_spi.hpp
 *
 *  Description:
 *    Orbit ESC SPI Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SPI_HPP
#define ORBIT_ESC_SPI_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/spi>


namespace Orbit::SPI
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the SPI driver subsystem
   */
  void powerUp();

}  // namespace Orbit::SPI

#endif  /* !ORBIT_ESC_SPI_HPP */
