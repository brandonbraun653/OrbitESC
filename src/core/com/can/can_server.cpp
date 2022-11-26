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
#include <src/core/com/can/can_async_message.hpp>
#include <src/core/com/can/can_message.hpp>
#include <src/core/com/can/can_periodic_message.hpp>
#include <src/core/com/can/can_server.hpp>

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
            ping.unpack( frame );

            this->receive( ping );
          }
          break;

          case Message::MSG_SET_SYSTEM_MODE: {
            Message::SetSystemMode set_system_mode;
            set_system_mode.unpack( frame );

            this->receive( set_system_mode );
          }
          break;

          case Message::MSG_EMERGENCY_HALT: {
            Message::EmergencyHalt emergency_halt;
            emergency_halt.unpack( frame );

            this->receive( emergency_halt );
          }
          break;

          case Message::MSG_SET_MOTOR_SPEED: {
            Message::SetMotorSpeed set_motor_speed;
            set_motor_speed.unpack( frame );

            this->receive( set_motor_speed );
          }
          break;

          case Message::MSG_SYSTEM_RESET: {
            Message::SystemReset system_reset;
            system_reset.unpack( frame );

            this->receive( system_reset );
          }
          break;

          default:
            LOG_ERROR( "CANServer received unhandled message: 0x%X\r\n", frame.id );
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
