/******************************************************************************
 *  File Name:
 *    orbit_usart.hpp
 *
 *  Description:
 *    Orbit ESC USART Driver
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_USART_HPP
#define ORBIT_ESC_USART_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/usart>
#include <Chimera/serial>


namespace Orbit::USART
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern Chimera::Serial::Driver_rPtr SerialDriver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
    /**
   * @brief Powers up the USART driver subsystem
   */
  void powerUp();

}  // namespace Orbit::USART

#endif  /* !ORBIT_ESC_USART_HPP */
