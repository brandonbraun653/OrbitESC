/******************************************************************************
 *  File Name:
 *    serial_message_intf.hpp
 *
 *  Description:
 *    Common serial message interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SERIAL_MESSAGE_INTF_HPP
#define ORBIT_SERIAL_MESSAGE_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "serial_interface.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <Chimera/common>
#include <Chimera/serial>
#include <etl/message.h>
#include <etl/array.h>

namespace Orbit::Serial::Message
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum Id : etl::message_id_t
  {
    MSG_PING_CMD  = 1,  /**< Simple PING to see if the node is alive */

  };

  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Ping;

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  template<etl::message_id_t MSG_ID, typename PayloadType, const size_t PayloadSize, const pb_msgdesc_t * PayloadFields>
  class MessageExt : public etl::message<MSG_ID>
  {
  public:
    PayloadType payload;

    /**
     * @brief Resets the message payload to its default value
     */
    void reset()
    {
      bytesWritten = 0;
      ioBuffer.fill( 0 );
    }

    bool encode()
    {
      return this->encode( payload );
    }

    bool encode( const PayloadType &message )
    {
      pb_ostream_t stream = pb_ostream_from_buffer( ioBuffer.data(), ioBuffer.size() );
      bool         result = pb_encode( &stream, PayloadFields, &message );
      bytesWritten        = stream.bytes_written;
      return result;
    }

    bool decode( const void *const src, const size_t length )
    {
      return this->decode( payload, src, length );
    }

    /**
     * @brief Decodes a buffer
     *
     * @param dst
     * @param src
     * @param length
     * @return true
     * @return false
     */
    bool decode( PayloadType &dst, const void *const src, const size_t length )
    {
      /*-----------------------------------------------------------------------
      Input Protection
      -----------------------------------------------------------------------*/
      if( !src || ( length < PayloadSize ) )
      {
        RT_DBG_ASSERT( false );
        return false;
      }


      pb_istream_t stream = pb_istream_from_buffer( reinterpret_cast<const pb_byte_t*>( src ), PayloadSize );
      return pb_decode( &stream, PayloadFields, &dst );
    }

    /**
     * @brief Sends the encoded message data over a serial link
     *
     * @param serial     Serial port to write to
     * @return Chimera::Status_t
     */
    Chimera::Status_t send( Chimera::Serial::Driver_rPtr serial )
    {
      RT_DBG_ASSERT( serial );
      if ( ( bytesWritten == 0 ) || ( bytesWritten > PayloadSize ) )
      {
        return Chimera::Status::FAIL;
      }

      return serial->write( ioBuffer.data(), bytesWritten );
    }

  protected:
    uint16_t bytesWritten;
    etl::array<uint8_t, PayloadSize> ioBuffer;
  };



  class Ping : public MessageExt<MSG_PING_CMD, PingMessage, PingMessage_size, PingMessage_fields>
  {
  public:
    // using PayloadType                          = PingMessage;
    // static constexpr PingMessage PayloadInit   = PingMessage_init_zero;
    // static constexpr size_t      PayloadSize   = PingMessage_size;
    //const pb_msgdesc_t *const    PayloadFields = PingMessage_fields;
  };

}  // namespace Orbit::Serial::Message

#endif  /* !ORBIT_SERIAL_MESSAGE_INTF_HPP */
