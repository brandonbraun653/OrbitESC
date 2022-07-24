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
