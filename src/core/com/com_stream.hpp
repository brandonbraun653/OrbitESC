/******************************************************************************
 *  File Name:
 *    com_stream.hpp
 *
 *  Description:
 *    Streaming interface to publish data on-demand to the remote host. This is
 *    intended to be a highly configurable "build-a-stream" type of interface
 *    where each stream item is a single data point. The idea is that a user
 *    can request multiple streams with different rates without having to
 *    change system message definitions.
 *
 *    Generally speaking this is a lightweight interface to quickly inspect the
 *    system. If it's required to group/send multiple data points together,
 *    consider adding a dedicated message type and transmit functionality.
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_COM_APP_TX_HPP
#define ORBIT_COM_APP_TX_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <src/core/com/proto/system_data.pb.h>
#include <src/core/com/com_stream_tasks.hpp>

namespace Orbit::COM::Stream
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  /**
   * @brief Current state of a data stream
   */
  struct StreamState
  {
    bool     active;          /**< If the stream is active */
    uint32_t deadline_ms;     /**< Deadline to enqueue the next chunk */
    uint32_t last_enqueue_ms; /**< Last time the stream was enqueued */
  };

  /**
   * @brief Data allowed to be transmitted to the host
   */
  union StreamData
  {
    bool bool_value;
    float float32;
    uint32_t uint32;
    uint16_t uint16;
    uint8_t uint8;
    int32_t int32;
    int16_t int16;
    int8_t int8;
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the data stream module
   * @return void
   */
  void powerUp();

  /**
   * @brief Get the state of a stream
   *
   * @param id ID of the stream to get the state of
   * @return const StreamState &
   */
  const StreamState &getState( const SystemDataId id );

  /**
   * @brief Process a request to configure stream behavior.
   *
   * @param request Request to process
   * @return True if the request was handled correctly, false otherwise
   */
  bool processRequest( const StreamRequestMessage &request );

  /**
   * @brief Enqueue a chunk of data to be transmitted.
   * @warning Do not call this from an ISR. Use the "fromISR" version instead.
   *
   * @param id ID of the stream to apply the data to
   * @param data Data to be transmitted
   * @return True if the chunk was successfully enqueued, false otherwise
   */
  bool enqueue( const SystemDataId id, const StreamData &&data );

  /**
   * @brief Enqueue a chunk of data to be transmitted.
   * @warning This should be called **only** from an ISR.
   *
   * @param id ID of the stream to apply the data to
   * @param data Data to be transmitted
   * @return True if the chunk was successfully enqueued, false otherwise
   */
  bool enqueueFromISR( const SystemDataId id, const StreamData &&data );

}    // namespace Orbit::COM::Stream

#endif /* !ORBIT_COM_APP_TX_HPP */
