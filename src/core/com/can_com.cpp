/******************************************************************************
 *  File Name:
 *    can_com.cpp
 *
 *  Description:
 *    Communication message implementation details
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/can>
#include <Chimera/common>
#include <src/core/com/can_com.hpp>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::CAN::BasicFrame defaultFrame()
  {
    Chimera::CAN::BasicFrame msg;
    msg.clear();
    msg.idMode    = Chimera::CAN::IdType::STANDARD;
    msg.frameType = Chimera::CAN::FrameType::DATA;

    return msg;
  }


  Chimera::Status_t pack( const uint8_t msg, const void *const data, const size_t size, Chimera::CAN::BasicFrame &frame )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !data || !size || ( size > Chimera::CAN::MAX_PAYLOAD_LENGTH ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    bool valid_msg = false;
    switch ( msg )
    {
      case Message::SystemTick::CanID:
        valid_msg = ( sizeof( Message::SystemTick::PayloadType ) == size );
        break;

      case Message::Ping::CanID:
        valid_msg = ( sizeof( Message::Ping::PayloadType ) == size );
        break;

      default:
        LOG_ERROR( "Tried to pack unhandled CAN message: 0x%X\r\n", msg );
        break;
    };

    if ( !valid_msg )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Build the frame
    -------------------------------------------------------------------------*/
    frame.clear();
    frame.id         = msg;
    frame.idMode     = Chimera::CAN::IdType::STANDARD;
    frame.frameType  = Chimera::CAN::FrameType::DATA;
    frame.dataLength = size;
    memcpy( frame.data, data, size );

    return Chimera::Status::OK;
  }


  Chimera::Status_t unpack( const Chimera::CAN::BasicFrame &frame, void *const data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    bool valid_msg = false;
    switch ( frame.id )
    {
      case Message::SystemTick::CanID:
        valid_msg = ( sizeof( Message::SystemTick::PayloadType ) == size );
        break;

      case Message::Ping::CanID:
        valid_msg = ( sizeof( Message::Ping::PayloadType ) == size );
        break;

      default:
        LOG_ERROR( "Tried to unpack unhandled CAN message: 0x%X\r\n", frame.id );
        break;
    };

    if ( !valid_msg || !data || !size || ( size < frame.dataLength ) )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Copy out the data
    -------------------------------------------------------------------------*/
    memcpy( data, frame.data, frame.dataLength );
    return Chimera::Status::OK;
  }

}    // namespace Orbit::CAN
