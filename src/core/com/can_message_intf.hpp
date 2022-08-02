/******************************************************************************
 *  File Name:
 *    can_message_intf.hpp
 *
 *  Description:
 *    Common CAN message interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_MESSAGE_INTF_HPP
#define ORBIT_CAN_MESSAGE_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>
#include <Chimera/common>
#include <cstdint>
#include <etl/message.h>
#include <src/core/hw/orbit_can.hpp>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Simple meta-information header for CAN messages
   */
  struct Header
  {
    uint8_t nodeId : 3; /**< Describes the ESC that generated a message */
    uint8_t size : 3;   /**< Number of bytes in the message */
    uint8_t _pad : 2;   /**< Reserved for future use */
  };
  static_assert( sizeof( Header ) == sizeof( uint8_t ) );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  template<class... T>
  static constexpr size_t calcPeriodicMessageTypes()
  {
    return ( 0 + ... + T::_foldExprPeriodicCounter() );
  }


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief CRTP base class for CAN message routers
   *
   * @tparam Base         CRTP base class
   * @tparam ARB_ID       Arbitration ID of the message
   * @tparam PERIOD_MS    Period of the message in milliseconds
   */
  template<typename Base, etl::message_id_t ARB_ID, uint32_t PERIOD_MS = 0>
  class Attributes : public etl::message<ARB_ID>
  {
  public:

    static constexpr size_t payloadSize()
    {
      static_assert( sizeof( typename Base::Payload ) <= Chimera::CAN::MAX_PAYLOAD_LENGTH );
      return sizeof( typename Base::Payload );
    }

    static constexpr uint32_t period()
    {
      return PERIOD_MS;
    }

    static constexpr bool isPeriodic()
    {
      return PERIOD_MS != 0;
    }

    static constexpr uint32_t _foldExprPeriodicCounter()
    {
      return PERIOD_MS ? 1u : 0u;
    }

    /**
     * @brief Optional method for periodic messages to update their data
     */
    virtual void update( void )
    {
      if constexpr ( PERIOD_MS != 0 )
      {
        RT_HARD_ASSERT( false );    // Force implementation of update function
      }
    };

    /**
     * @brief Resets the message payload to its default value
     */
    void reset()
    {
      memset( &( static_cast<Base *>( this )->payload ), 0, payloadSize() );
    }

    /**
     * @brief Packs a frame with the current data
     *
     * @param frame   Output frame to pack
     * @return Chimera::Status_t
     */
    Chimera::Status_t pack( Chimera::CAN::BasicFrame &frame )
    {
      /*-------------------------------------------------------------------------
      Build the frame
      -------------------------------------------------------------------------*/
      frame.clear();
      frame.id         = ARB_ID;
      frame.idMode     = Chimera::CAN::IdType::STANDARD;
      frame.frameType  = Chimera::CAN::FrameType::DATA;
      frame.dataLength = payloadSize();
      memcpy( frame.data, &( static_cast<Base *>( this )->payload ), payloadSize() );

      return Chimera::Status::OK;
    }

    /**
     * @brief Unpacks a frame into the message payload
     *
     * @param frame   Input frame to unpack
     * @return Chimera::Status_t
     */
    Chimera::Status_t unpack( const Chimera::CAN::BasicFrame &frame )
    {
      /*-----------------------------------------------------------------------
      Input Protection
      -----------------------------------------------------------------------*/
      if ( ( frame.dataLength != payloadSize() ) || ( frame.id != static_cast<uint32_t>( ARB_ID ) ) )
      {
        return Chimera::Status::FAIL;
      }

      /*-----------------------------------------------------------------------
      Copy out the data
      -----------------------------------------------------------------------*/
      memcpy( &( static_cast<Base *>( this )->payload ), frame.data, frame.dataLength );
      return Chimera::Status::OK;
    }

    /**
     * @brief Sends the message over the CAN bus
     *
     * @param driver Driver to use for sending the message
     * @return Chimera::Status_t
     */
    Chimera::Status_t send( Chimera::CAN::Driver_rPtr driver )
    {
      Chimera::CAN::BasicFrame frame;
      Chimera::Status_t        status;

      status = pack( frame );
      if ( status != Chimera::Status::OK )
      {
        return status;
      }

      RT_DBG_ASSERT( driver );
      return driver->send( frame );
    }

  };
}    // namespace Orbit::CAN::Message

#endif /* !ORBIT_CAN_MESSAGE_INTF_HPP */
