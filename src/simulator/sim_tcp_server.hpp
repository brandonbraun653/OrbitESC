/******************************************************************************
 *  File Name:
 *    sim_tcp_server.hpp
 *
 *  Description:
 *    Matlab TCP/IP bridge for simulator builds.
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SIM_TCP_SERVER_HPP
#define ORBIT_ESC_SIM_TCP_SERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Orbit::Sim::Matlab
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Payload received from Matlab.
   *
   * The payload mirrors a fixed-size Matlab array that is transmitted as raw
   * binary data over the TCP socket. Adjust the number of floats to match the
   * Matlab configuration when integrating with a specific model.
   */
  struct RxMessage
  {
    double value;
  };


  /**
   * @brief Payload transmitted to Matlab.
   */
  struct TxMessage
  {
    double value;
  };

  static_assert( std::is_standard_layout_v<RxMessage>, "RxMessage must be POD" );
  static_assert( std::is_standard_layout_v<TxMessage>, "TxMessage must be POD" );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Starts the Matlab TCP/IP server thread.
   *
   * @param port  TCP port to listen on. Defaults to 55001 to avoid conflicts
   *              with other simulator services.
   * @return true Server thread started or already running
   * @return false Failed to launch the server thread
   */
  bool startServer( uint16_t port = 55001 );


  /**
   * @brief Stops the Matlab TCP/IP server thread.
   */
  void stopServer();


  /**
   * @brief Pushes a message to Matlab. The message is transmitted on the next
   *        write-ready cycle for the active client connection.
   */
  void pushMessage( const TxMessage &message );


  /**
   * @brief Retrieves the most recent message received from Matlab.
   *
   * @param message  Output buffer for the message
   * @return true    A fresh message was available and copied into @p message
   * @return false   No new message has been received since the last call
   */
  bool getLastMessage( RxMessage &message );


  /**
   * @brief Indicates whether the server worker thread is active.
   */
  bool isRunning();


  /**
   * @brief Indicates whether a Matlab client is currently connected.
   */
  bool isClientConnected();

}    // namespace Orbit::Sim::Matlab

#endif /* !ORBIT_ESC_SIM_TCP_SERVER_HPP */
