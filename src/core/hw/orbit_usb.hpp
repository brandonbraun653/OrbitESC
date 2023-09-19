/******************************************************************************
 *  File Name:
 *    orbit_usb.hpp
 *
 *  Description:
 *    Orbit ESC USB driver interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_USB_HPP
#define ORBIT_ESC_USB_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Orbit::USB
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the USB driver subsystem
   */
  void powerUp();

}    // namespace Orbit::USB

#endif /* !ORBIT_ESC_USB_HPP */
