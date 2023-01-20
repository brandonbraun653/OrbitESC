/******************************************************************************
 *  File Name:
 *    serial_server.hpp
 *
 *  Description:
 *    Server interface for serial commands
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SERIAL_SERVER_HPP
#define ORBIT_ESC_SERIAL_SERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <Chimera/serial>
#include <etl/message_bus.h>
#include <etl/circular_buffer.h>


namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief A simple dispatch server to process incoming serial messages
   *
   * Decodes COBS messages on the bus, pushes them through protocol buffers
   * decoding, and invokes the registered handler if any exists.
   */
  class DispatchServer : public etl::message_bus<8>
  {
  public:
    DispatchServer();
    ~DispatchServer();

    /**
     * @brief Initialize the Serial bus dispatch server
     *
     * @param channel     Which serial channel to listen on
     * @param msg_buffer  Message buffer to use to cache messages
     * @return Chimera::Status_t
     */
    Chimera::Status_t initialize( const Chimera::Serial::Channel channel, etl::icircular_buffer<uint8_t> &msg_buffer );

    /**
     * @brief Process the server runtime
     */
    void process();

  private:
    Chimera::Serial::Driver_rPtr    mSerial;    /**< Serial port to communicate with */
    etl::icircular_buffer<uint8_t> *mRXBuffer;  /**< Input ring buffer for accumulating messages */

    /**
     * @brief Processes any queued messages in the internal buffer
     */
    void dispatchMessages();
  };
}  // namespace Orbit::Serial

#endif  /* !ORBIT_ESC_SERIAL_SERVER_HPP */
