/******************************************************************************
 *  File Name:
 *    serial_runtime.cpp
 *
 *  Description:
 *    Serial bus processing
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/assert>
#include <etl/queue_spsc_atomic.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_router.hpp>
#include <src/core/com/serial/serial_server.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_usart.hpp>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/com/serial/serial_config.hpp>


namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using SystemDataQueue =
      etl::queue_spsc_atomic<Message::SysData, 64, etl::memory_model::MEMORY_MODEL_SMALL>;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static etl::circular_buffer<uint8_t, 1024> s_msg_buffer;
  static Orbit::Serial::DispatchServer       s_server;

  /*---------------------------------------------------------------------------
  Router Declarations
  ---------------------------------------------------------------------------*/
  static Router::PingRouter       s_ping_router;
  static Router::ParamIORouter    s_param_router;
  static Router::SysCtrlRouter    s_sys_ctrl_router;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Handle requests to GET a parameter value
   * @param msg Message request
   */
  static void handle_get( const Message::ParamIO &msg )
  {
    using namespace Orbit::Data;

    LOG_TRACE( "GET request for param: %d", msg.raw.id );

    /*-------------------------------------------------------------------------
    Ensure the parameter ID is valid
    -------------------------------------------------------------------------*/
    const ParamId id = static_cast<ParamId>( msg.raw.id );
    if ( !Param::exists( id ) )
    {
      sendAckNack( false, msg.raw.header, StatusCode_INVALID_PARAM );
      return;
    }

    /*-------------------------------------------------------------------------
    Prepare the response
    -------------------------------------------------------------------------*/
    Message::ParamIO reply;
    reply.raw.header   = msg.raw.header;
    reply.raw.id       = msg.raw.id;
    reply.raw.type     = Param::type( id );
    reply.raw.has_type = true;
    reply.raw.has_data = true;
    memset( reply.raw.data.bytes, 0, sizeof( reply.raw.data.bytes ) );

    /*-------------------------------------------------------------------------
    Copy out the serialized data
    -------------------------------------------------------------------------*/
    const ssize_t read_size = Param::read( id, reply.raw.data.bytes, sizeof( reply.raw.data.bytes ) );
    if ( read_size < 0 )
    {
      sendAckNack( false, msg.raw.header, StatusCode_REQUEST_FAILED );
      return;
    }

    reply.raw.data.size = static_cast<pb_size_t>( read_size );

    /*-------------------------------------------------------------------------
    Ship the response on the wire
    -------------------------------------------------------------------------*/
    if( !Message::encode( &reply.state ) || ( Message::send( &reply.state, Config::getCommandPort() ) != Chimera::Status::OK ) )
    {
      LOG_ERROR( "Failed to send response to GET request" );
    }

    LOG_TRACE( "GET request complete" );
  }


  /**
   * @brief Handle requests to PUT a parameter value
   * @param msg Message request
   */
  static void handle_put( const Message::ParamIO &msg )
  {
    using namespace Orbit::Data;
    LOG_TRACE( "PUT request for param: %d", msg.raw.id );

    /*-------------------------------------------------------------------------
    Ensure the parameter ID is valid
    -------------------------------------------------------------------------*/
    const ParamId id = static_cast<ParamId>( msg.raw.id );
    if ( !Param::exists( id ) )
    {
      sendAckNack( false, msg.raw.header, StatusCode_INVALID_PARAM );
      return;
    }

    /*-------------------------------------------------------------------------
    Load the data into the cache
    -------------------------------------------------------------------------*/
    if ( msg.raw.has_data && Param::write( id, msg.raw.data.bytes, msg.raw.data.size ) )
    {
      sendAckNack( true, msg.raw.header );
    }
    else
    {
      sendAckNack( false, msg.raw.header, StatusCode_REQUEST_FAILED );
    }

    LOG_TRACE( "PUT request complete" );
  }


  /**
   * @brief Handle requests to LOAD data from disk
   * @param msg Message request
   */
  static void handle_load( const Message::ParamIO &msg )
  {
    LOG_TRACE( "LOAD param request" );
    sendAckNack( Data::Param::load(), msg.raw.header );
  }


  /**
   * @brief Handle requests to SYNC changes to disk
   * @param msg Message request
   */
  static void handle_sync( const Message::ParamIO &msg )
  {
    LOG_TRACE( "SYNC param request" );
    sendAckNack( Data::Param::flush(), msg.raw.header );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initRuntime()
  {
    /*-------------------------------------------------------------------------
    Initialize the core server
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_server.initialize( Config::getCommandPort(), s_msg_buffer ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Register the routers to handle incoming messages
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_server.subscribe( s_ping_router ) );
    RT_HARD_ASSERT( s_server.subscribe( s_param_router ) );
    RT_HARD_ASSERT( s_server.subscribe( s_sys_ctrl_router ) );
  }


  void processSerial()
  {
    /*-------------------------------------------------------------------------
    Run the message processing loop
    -------------------------------------------------------------------------*/
    s_server.process();
  }


  void handleParamIOEvent()
  {
    while ( Router::ParamIOEventQueue.size() )
    {
      Message::ParamIO msg = Router::ParamIOEventQueue.front();
      Router::ParamIOEventQueue.pop();

      switch ( msg.raw.header.subId )
      {
        case ParamIOSubId_GET:
          handle_get( msg );
          break;

        case ParamIOSubId_SET:
          handle_put( msg );
          break;

        case ParamIOSubId_LOAD:
          handle_load( msg );
          break;

        case ParamIOSubId_SYNC:
          handle_sync( msg );
          break;

        default:
          LOG_ERROR( "Unhandled ParamIO subId: %d", msg.raw.header.subId );
          break;
      }
    }
  }

}    // namespace Orbit::Serial
