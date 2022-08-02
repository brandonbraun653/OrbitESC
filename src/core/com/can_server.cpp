/******************************************************************************
 *  File Name:
 *    can_server.cpp
 *
 *  Description:
 *    CAN bus dispatch communication server
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/can>
#include <src/core/com/can_server.hpp>
#include <src/core/com/can_message.hpp>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  Server::Server() : mCANBus( nullptr )
  {
  }


  Server::~Server()
  {
  }


  Chimera::Status_t Server::initialize( const Chimera::CAN::Channel channel )
  {
    mCANBus = Chimera::CAN::getDriver( channel );
    RT_HARD_ASSERT( mCANBus );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Server::transmit( const Chimera::CAN::BasicFrame &frame )
  {
    if ( !mCANBus )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    return mCANBus->send( frame );
  }


  Chimera::Status_t Server::stream( const uint8_t node, const StreamId id, const void *const data, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Ensure safe access
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( *this );

    if ( !mCANBus || mStream.data )
    {
      return Chimera::Status::NOT_READY;
    }

    /*-------------------------------------------------------------------------
    Initialize the stream
    -------------------------------------------------------------------------*/
    mStream.id     = id;
    mStream.data   = data;
    mStream.size   = size;
    mStream.offset = 0;
    mStream.node   = node;

    return Chimera::Status::OK;
  }


  void Server::cancelStream()
  {
    Chimera::Thread::LockGuard _lck( *this );
    mStream.clear();
  }


  void Server::processRTX()
  {
    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if ( !mCANBus )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Pull any new messages off the queue and dispatch to their handlers
    -------------------------------------------------------------------------*/
    size_t available_frames = mCANBus->available();
    while ( available_frames )
    {
      Chimera::CAN::BasicFrame frame;
      Chimera::Status_t        result = mCANBus->receive( frame );

      if ( Chimera::Status::OK == result )
      {
        switch ( frame.id )
        {
          case Message::MSG_PING: {
            Message::Ping ping;
            ping.pack( frame );

            this->receive( ping );
          }
          break;

          default:
            LOG_ERROR( "Unhandled CAN message: 0x%X", frame.id );
            break;
        }
      }
      else
      {
        LOG_ERROR( "Failed CAN frame reception with error code: %d\r\n", result );
      }

      available_frames = mCANBus->available();
    }

    /*-------------------------------------------------------------------------
    Process periodic transmit events
    -------------------------------------------------------------------------*/
    for ( auto &event : mPeriodicEvents )
    {
      event.poll();
    }

    /*-------------------------------------------------------------------------
    Process streaming data
    -------------------------------------------------------------------------*/
    this->runStreamer();
  }


  Chimera::Status_t Server::registerPeriodic( Chimera::Function::Opaque method, const size_t rate )
  {
    Chimera::Scheduler::Polled event;
    event.periodic( method, rate );

    if ( mPeriodicEvents.size() < mPeriodicEvents.max_size() )
    {
      mPeriodicEvents.push_back( event );
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FULL;
    }
  }


  void Server::runStreamer()
  {
    /*-------------------------------------------------------------------------
    Ensure there is actually data to stream
    -------------------------------------------------------------------------*/
    Chimera::Thread::LockGuard _lck( *this );
    const size_t               remaining_bytes = mStream.size - mStream.offset;

    if ( !mStream.data || !remaining_bytes )
    {
      mStream.clear();
      return;
    }

    /*-------------------------------------------------------------------------
    Collect a few common variables
    -------------------------------------------------------------------------*/
    Chimera::CAN::BasicFrame canFrame;
    const size_t             bytes_to_send = std::min( remaining_bytes, sizeof( Message::StreamData::Payload::data ) );

    /*-----------------------------------------------------------------------
    First message? Send the bookend message to kick things off.
    -----------------------------------------------------------------------*/
    if ( !mStream.started )
    {
      Message::StreamBookend bookend;

      bookend.reset();
      bookend.payload.src.nodeId = mStream.node;
      bookend.payload.type       = mStream.id;
      bookend.payload.bytes      = mStream.size;

      if ( ( bookend.pack( canFrame ) == Chimera::Status::OK ) && ( mCANBus->send( canFrame ) == Chimera::Status::OK ) )
      {
        mStream.started = true;
        mStream.offset  = 0;
        LOG_INFO( "CAN streaming started\r\n" );
      }
      else
      {
        LOG_ERROR( "CAN streaming failed to start\r\n" );
      }

      return;
    }

    /*-------------------------------------------------------------------------
    Pack the next chunk of data to send
    -------------------------------------------------------------------------*/
    Message::StreamData data;

    data.reset();
    data.payload.src.nodeId = mStream.node;
    data.payload.type       = mStream.id;
    data.payload.frame      = mStream.frameNum;
    memcpy( data.payload.data, ( ( const uint8_t * )mStream.data ) + mStream.offset, bytes_to_send );

    if ( ( data.pack( canFrame ) == Chimera::Status::OK ) && ( mCANBus->send( canFrame ) == Chimera::Status::OK ) )
    {
      mStream.attempts = 0;
      mStream.offset += bytes_to_send;
      mStream.frameNum++;

      if ( mStream.offset >= mStream.size )
      {
        LOG_INFO( "CAN streamed %d bytes\r\n", mStream.size );
        mStream.clear();
      }
    }
    else
    {
      mStream.attempts++;
      if ( mStream.attempts >= 5 )
      {
        mStream.clear();
        LOG_ERROR( "CAN stream failed to send\r\n" );
      }
    }
  }
}    // namespace Orbit::CAN
