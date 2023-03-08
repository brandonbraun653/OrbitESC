/******************************************************************************
 *  File Name:
 *    system_control.hpp
 *
 *  Description:
 *    System control interface for the serial message router
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TESTING_SYSTEM_CONTROL_HPP
#define ORBIT_ESC_TESTING_SYSTEM_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/serial/serial_async_message.hpp>


namespace Orbit::Testing::SystemControl
{
  /**
   * @brief Handles an arriving system control message
   *
   * @param msg     Message to process
   * @return true   Message was handled
   * @return false  Message was not handled
   */
  bool handleMessage( const Serial::Message::SysCtrl &msg );

}  // namespace Orbit::Testing::SystemControl

#endif  /* !ORBIT_ESC_TESTING_SYSTEM_CONTROL_HPP */
