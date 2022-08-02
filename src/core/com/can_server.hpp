/******************************************************************************
 *  File Name:
 *    can_server.hpp
 *
 *  Description:
 *    CAN bus dispatch communication server
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_DISPATCH_SERVER_HPP
#define ORBIT_CAN_DISPATCH_SERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>
#include <Chimera/scheduler>
#include <Chimera/thread>
#include <etl/message_bus.h>
#include <etl/vector.h>
#include <src/core/com/can_message.hpp>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using StreamId       = uint8_t;
  using PeriodicVector = etl::vector<Chimera::Scheduler::Polled, Message::numPeriodicMessageTypes()>;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct StreamCB
  {
    bool        started;  /**< Started transmission flag */
    const void *data;     /**< Data to transmit */
    size_t      size;     /**< Number of bytes in the buffer */
    size_t      offset;   /**< Current offset into the buffer */
    StreamId    id;       /**< ID of the stream */
    uint8_t     node;     /**< Node ID of the streamer */
    uint8_t     frameNum; /**< Current frame ID of the streamer */
    uint8_t     attempts; /**< Number of attempts to send the frame */

    void clear()
    {
      started  = false;
      data     = nullptr;
      size     = 0;
      offset   = 0;
      id       = 0;
      node     = 0;
      frameNum = 0;
      attempts = 0;
    }
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Core communications server for processing CAN bus messages
   *
   * This class dispatches CAN bus messages to the appropriate handlers, TX's
   * asynchronous frames, and handles scheduling periodic messages.
   */
  class Server : public etl::message_bus<8>, public Chimera::Thread::Lockable<Server>
  {
  public:
    Server();
    ~Server();

    /**
     * @brief Initialize the CAN bus dispatch server
     *
     * @return Chimera::Status_t
     */
    Chimera::Status_t initialize( const Chimera::CAN::Channel channel );

    /**
     * @brief Periodically process RX/TX messages
     */
    void processRTX();

    /**
     * @brief Transmit a message directly on the CAN bus
     *
     * @param frame   Frame to transmit
     * @return Chimera::Status_t
     */
    Chimera::Status_t transmit( const Chimera::CAN::BasicFrame &frame );

    /**
     * @brief Streams a block of data that is larger than a single CAN frame
     *
     * @param node  Which CAN node is streaming the data
     * @param type  Associated message type. This is independent of the arbitration ID.
     * @param data  Pointer to the data to stream. This must be a contiguous block of data.
     * @param size  Number of bytes to stream.
     * @return Chimera::Status::OK            The message successfully started streaming
     * @return Chimera::Status::NOT_READY     Another message is already streaming
     */
    Chimera::Status_t stream( const uint8_t node, const StreamId type, const void *const data, const size_t size );

    /**
     * @brief Cancels a stream if one is in-progress
     */
    void cancelStream();

    /**
     * @brief Registers an update function to be called periodically
     *
     * @param method  Method to call periodically
     * @param rate    Rate to call the method at in milliseconds
     * @return Chimera::Status_t
     */
    Chimera::Status_t registerPeriodic( Chimera::Function::Opaque method, const size_t rate );

  protected:
    /**
     * @brief If a stream is in progress, this method will transmit the next frame
     */
    void runStreamer();

  private:
    friend Chimera::Thread::Lockable<Server>;

    Chimera::CAN::Driver_rPtr mCANBus;         /**< CAN bus driver */
    StreamCB                  mStream;         /**< Tracks the current stream operation */
    PeriodicVector            mPeriodicEvents; /**< Vector of periodic events */
  };

}    // namespace Orbit::CAN

#endif /* !ORBIT_CAN_DISPATCH_SERVER_HPP */
