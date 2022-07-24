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
#include <src/core/com/can_server.hpp>
#include <src/core/com/can_message_intf.hpp>

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
}    // namespace Orbit::CAN
