/******************************************************************************
 *  File Name:
 *    tsk_cdc.hpp
 *
 *  Description:
 *    USB CDC task interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_USB_CDC_TASK_HPP
#define ORBIT_ESC_USB_CDC_TASK_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>


namespace Orbit::Tasks::USB::CDC
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        STACK    = STACK_BYTES( 8192 );
  static constexpr std::string_view              NAME     = "cdc";
  static constexpr Chimera::Thread::TaskPriority PRIORITY = 4;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for usb CDC task
   *
   * @param arg   Unused
   */
  void USBCDCThread( void *arg );

}    // namespace Orbit::Tasks::USB::CDC

#endif /* !ORBIT_ESC_USB_CDC_TASK_HPP */
