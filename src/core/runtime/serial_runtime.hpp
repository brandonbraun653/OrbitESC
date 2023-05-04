/******************************************************************************
 *  File Name:
 *    serial_runtime.hpp
 *
 *  Description:
 *    Serial runtime operations for processing debug interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_RUNTIME_HPP
#define ORBIT_SERIAL_RUNTIME_HPP

#include <src/core/com/serial/serial_async_message.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the serial runtime infrastructure
   */
  void initRuntime();

  /**
   * @brief Periodic processing of the serial bus
   */
  void processSerial();

  /**
   * @brief High level handler for parameter IO requests
   */
  void handleParamIOEvent();

  /**
   * @brief Enqueues a data message to be sent out over the serial bus
   * @note This is a non-blocking call that's safe to call from an ISR
   *
   * @param msg The data message to send
   */
  void publishDataMessage( const Message::SysData &msg );

}  // namespace Orbit::Serial

#endif  /* !ORBIT_SERIAL_RUNTIME_HPP */
