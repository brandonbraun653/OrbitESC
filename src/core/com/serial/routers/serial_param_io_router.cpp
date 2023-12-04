/******************************************************************************
 *  File Name:
 *    serial_param_io_router.cpp
 *
 *  Description:
 *    Router for handling parameter IO messages
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_router.hpp>
#include <src/core/hw/orbit_usart.hpp>
#include <src/core/tasks.hpp>

namespace Orbit::Serial::Router
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  ParamIOQueue ParamIOEventQueue;


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  ParamIORouter::ParamIORouter() : message_router( Message::MSG_PARAM_IO )
  {
  }


  void ParamIORouter::on_receive( const Message::ParamIO &msg )
  {
    using namespace Orbit::Tasks;

    /*-------------------------------------------------------------------------
    Ensure enough space to handle the message
    -------------------------------------------------------------------------*/
    if ( ParamIOEventQueue.full() )
    {
      LOG_ERROR( "Dropped ParamIO request: %d", msg.raw.header.uuid );
      return;
    }

    /*-------------------------------------------------------------------------
    Notify the DIO thread of the message
    -------------------------------------------------------------------------*/
    ParamIOEventQueue.push( msg );
    Chimera::Thread::sendTaskMsg( getTaskId( TASK_DIO ), TASK_MSG_PARAM_IO_EVENT, Chimera::Thread::TIMEOUT_DONT_WAIT );
  }


  void ParamIORouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }
}    // namespace Orbit::Serial::Router
