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
#include <Chimera/common>
#include <Chimera/scheduler>
#include <Chimera/thread>
#include <etl/message_bus.h>
#include <etl/vector.h>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using StreamId       = uint8_t;
  using PeriodicVector = etl::vector<Chimera::Scheduler::Polled, 7>;


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
     * @brief Registers an update function to be called periodically
     *
     * @param method  Method to call periodically
     * @param rate    Rate to call the method at in milliseconds
     * @return Chimera::Status_t
     */
    Chimera::Status_t registerPeriodic( Chimera::Function::Opaque method, const size_t rate );

  private:
    friend Chimera::Thread::Lockable<Server>;

    uint16_t                  mFrameCount;     /**< Total number of frames processed */
    Chimera::CAN::Driver_rPtr mCANBus;         /**< CAN bus driver */
    PeriodicVector            mPeriodicEvents; /**< Vector of periodic events */
  };

}    // namespace Orbit::CAN

#endif /* !ORBIT_CAN_DISPATCH_SERVER_HPP */
