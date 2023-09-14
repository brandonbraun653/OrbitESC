/******************************************************************************
 *  File Name:
 *    tsk_usb.hpp
 *
 *  Description:
 *    USB task interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_USB_TASK_HPP
#define ORBIT_ESC_USB_TASK_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <string>


namespace Orbit::Tasks::USB
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t                        STACK    = STACK_BYTES( 4096 );
  static constexpr std::string_view              NAME     = "usb";
  static constexpr Chimera::Thread::TaskPriority PRIORITY = 5;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Main thread for usb task
   *
   * @param arg   Unused
   */
  void USBThread( void *arg );

}    // namespace Orbit::Tasks::USB

#endif /* !ORBIT_ESC_USB_TASK_HPP */
