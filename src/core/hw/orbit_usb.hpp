/******************************************************************************
 *  File Name:
 *    orbit_usb.hpp
 *
 *  Description:
 *    Orbit ESC USB driver interface
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_USB_HPP
#define ORBIT_ESC_USB_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>
#include <etl/queue_spsc_locked.h>

namespace Orbit::USB
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers up the USB driver subsystem
   */
  void powerUp();

  /**
   * @brief Register a callback for when the USB device is connected
   *
   * @param callback Function to be called
   * @return True if the callback was registered
   */
  bool onConnect( Chimera::Function::Opaque &&callback );

  /**
   * @brief Register a callback for when the USB device is disconnected
   *
   * @param callback Function to be called
   * @return True if the callback was registered
   */
  bool onDisconnect( Chimera::Function::Opaque &&callback );

  /**
   * @brief Disables the USB interrupts for critical sections of code
   * @return void
   */
  void disableInterrupts();

  /**
   * @brief Re-enables the USB interrupts after a critical section
   * @return void
   */
  void enableInterrupts();

}    // namespace Orbit::USB

#endif /* !ORBIT_ESC_USB_HPP */
