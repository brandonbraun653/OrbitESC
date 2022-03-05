/******************************************************************************
 *  File Name:
 *    can_com.hpp
 *
 *  Description:
 *    Communication header for CAN bus
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_COM_HPP
#define ORBIT_CAN_COM_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstddef>
#include <Chimera/common>
#include <Chimera/can>
#include <src/core/com/can_com_intf.hpp>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Unique ID for a node in the ESC system
   */
  enum class NodeId : uint8_t
  {
    NODE_0,
    NODE_1,
    NODE_2,
    NODE_3,
    NODE_4,
    NODE_5,
    NODE_PC,

    NUM_SUPPORTED_NODES
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  namespace _Internal
  {
    /**
     * @brief Describes the ESC that generated a message
     */
    struct SystemID
    {
      NodeId  nodeId : 3; /**< Unique node */
      uint8_t _pad : 5;   /**< Reserved for future use */
    };
    static_assert( sizeof( SystemID ) == sizeof( uint8_t ) );


    /**
     * @brief Current system tick of the OrbitESC node
     *
     */
    struct PLSystemTick
    {
      SystemID src;  /**< System information */
      uint32_t tick; /**< Current tick of system */
    } __attribute__( ( packed ) );
    static_assert( sizeof( PLSystemTick ) <= Chimera::CAN::MAX_PAYLOAD_LENGTH );


    /**
     * @brief Very simple "ping" packet. Sent and received by all nodes.
     */
    struct PLPing
    {
      SystemID dst; /**< Destination system information */
      SystemID src; /**< Source system information */
    } __attribute__( ( packed ) );
    static_assert( sizeof( PLPing ) <= Chimera::CAN::MAX_PAYLOAD_LENGTH );
  }    // namespace _Internal

  /*---------------------------------------------------------------------------
  Message Data
  ---------------------------------------------------------------------------*/
  namespace Message
  {
    enum _Number : uint8_t
    {
      /*-----------------------------------------------------------------------
      Periodic Messages (Must go first)
      -----------------------------------------------------------------------*/
      MSG_PERIODIC_START = 0,
      MSG_SYSTEM_TICK    = MSG_PERIODIC_START,

      // Add more periodic messages here. Update "END" enum.
      MSG_PERIODIC_END,

      /*-----------------------------------------------------------------------
      Async Messages
      -----------------------------------------------------------------------*/
      MSG_ASYNC_START = MSG_PERIODIC_END,
      MSG_PING        = MSG_ASYNC_START,

      // Add more async messages here. Update "END" enum.
      MSG_ASYNC_END,

      /*-----------------------------------------------------------------------
      Summary of enumerated values
      -----------------------------------------------------------------------*/
      MSG_INVALID,
      NUM_SUPPORTED_MSG = MSG_ASYNC_END,
      NUM_PERIODIC_MSG  = MSG_PERIODIC_END - MSG_PERIODIC_START,
      NUM_ASYNC_MSG     = MSG_ASYNC_END - MSG_ASYNC_START
    };

    /*-------------------------------------------------------------------------
    Type Declarations
    -------------------------------------------------------------------------*/
    using SystemTick = MessageDefinition<_Internal::PLSystemTick, MSG_SYSTEM_TICK, 0x15, 1000>;
    using Ping       = MessageDefinition<_Internal::PLPing, MSG_PING, 0x10>;

    /*-------------------------------------------------------------------------
    Public Functions
    -------------------------------------------------------------------------*/
    static constexpr uint8_t periodicOffset( const uint8_t x )
    {
      return ( ( x >= MSG_PERIODIC_END ) || ( x < MSG_PERIODIC_START ) ) ? MSG_INVALID : ( x - MSG_PERIODIC_START );
    }

    static constexpr uint8_t asyncOffset( const uint8_t x )
    {
      return ( ( x >= MSG_ASYNC_END ) || ( x < MSG_ASYNC_START ) ) ? MSG_INVALID : ( x - MSG_ASYNC_START );
    }
  }    // namespace Message


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Builds an (empty) default frame for the project's communication style
   *
   * @return Chimera::CAN::BasicFrame
   */
  Chimera::CAN::BasicFrame defaultFrame();

  /**
   * @brief Packs a frame with the given data
   *
   * @param msg     Message being packed
   * @param data    Data payload, if any
   * @param size    Size of the data payload
   * @param frame   Output frame
   * @return Chimera::Status_t
   */
  Chimera::Status_t pack( const uint8_t msg, const void *const data, const size_t size, Chimera::CAN::BasicFrame &frame );

  /**
   * @brief Unpacks a frame into the given data buffer
   *
   * @param frame   Frame being unpacked
   * @param data    Output data buffer to write into
   * @param size    Size of the output data buffer
   * @return Chimera::Status_t
   */
  Chimera::Status_t unpack( const Chimera::CAN::BasicFrame &frame, void *const data, const size_t size );

}    // namespace Orbit::CAN

#endif /* !ORBIT_CAN_COM_HPP */
