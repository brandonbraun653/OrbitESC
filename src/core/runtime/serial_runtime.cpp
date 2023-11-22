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
  static SystemDataQueue                     s_sys_data_queue;

  /*---------------------------------------------------------------------------
  Router Declarations
  ---------------------------------------------------------------------------*/
  static Router::PingRouter       s_ping_router;
  static Router::ParamIORouter    s_param_router;
  static Router::SysCtrlRouter    s_sys_ctrl_router;
  static Router::SwitchModeRouter s_switch_mode_router;

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

    LOG_TRACE( "GET request for param: %d", msg.payload.id );

    /*-------------------------------------------------------------------------
    Ensure the parameter ID is valid
    -------------------------------------------------------------------------*/
    const ParamId id = static_cast<ParamId>( msg.payload.id );
    if ( !Param::exists( id ) )
    {
      sendAckNack( false, msg.payload.header, StatusCode_INVALID_PARAM );
      return;
    }

    /*-------------------------------------------------------------------------
    Prepare the response
    -------------------------------------------------------------------------*/
    Message::ParamIO reply;
    reply.payload.header   = msg.payload.header;
    reply.payload.id       = msg.payload.id;
    reply.payload.type     = Param::type( id );
    reply.payload.has_type = true;
    reply.payload.has_data = true;
    memset( reply.payload.data.bytes, 0, sizeof( reply.payload.data.bytes ) );

    /*-------------------------------------------------------------------------
    Copy out the serialized data
    -------------------------------------------------------------------------*/
    const ssize_t read_size = Param::read( id, reply.payload.data.bytes, sizeof( reply.payload.data.bytes ) );
    if ( read_size < 0 )
    {
      sendAckNack( false, msg.payload.header, StatusCode_REQUEST_FAILED );
      return;
    }

    reply.payload.data.size = static_cast<pb_size_t>( read_size );

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
    LOG_TRACE( "PUT request for param: %d", msg.payload.id );

    /*-------------------------------------------------------------------------
    Ensure the parameter ID is valid
    -------------------------------------------------------------------------*/
    const ParamId id = static_cast<ParamId>( msg.payload.id );
    if ( !Param::exists( id ) )
    {
      sendAckNack( false, msg.payload.header, StatusCode_INVALID_PARAM );
      return;
    }

    /*-------------------------------------------------------------------------
    Load the data into the cache
    -------------------------------------------------------------------------*/
    if ( msg.payload.has_data && Param::write( id, msg.payload.data.bytes, msg.payload.data.size ) )
    {
      sendAckNack( true, msg.payload.header );
    }
    else
    {
      sendAckNack( false, msg.payload.header, StatusCode_REQUEST_FAILED );
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
    sendAckNack( Data::Param::load(), msg.payload.header );
  }


  /**
   * @brief Handle requests to SYNC changes to disk
   * @param msg Message request
   */
  static void handle_sync( const Message::ParamIO &msg )
  {
    LOG_TRACE( "SYNC param request" );
    sendAckNack( Data::Param::flush(), msg.payload.header );
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
    RT_HARD_ASSERT( s_server.subscribe( s_switch_mode_router ) );
  }


  void processSerial()
  {
    /*-------------------------------------------------------------------------
    Run the message processing loop
    -------------------------------------------------------------------------*/
    s_server.process();

    /*-------------------------------------------------------------------------
    Flush any data messages that need to be sent
    -------------------------------------------------------------------------*/
    while( !s_sys_data_queue.empty() )
    {
      auto msg = s_sys_data_queue.front();
      Message::encode( &msg.state );

      if ( Chimera::Status::OK == Message::send( &msg.state, Config::getCommandPort() ) )
      {
        s_sys_data_queue.pop();
      }
      else
      {
        break;
      }
    }
  }


  void handleParamIOEvent()
  {
    while ( Router::ParamIOEventQueue.size() )
    {
      Message::ParamIO msg = Router::ParamIOEventQueue.front();
      Router::ParamIOEventQueue.pop();

      switch ( msg.payload.header.subId )
      {
        case SubId_SUB_MSG_PARAM_IO_GET:
          handle_get( msg );
          break;

        case SubId_SUB_MSG_PARAM_IO_SET:
          handle_put( msg );
          break;

        case SubId_SUB_MSG_PARAM_IO_LOAD:
          handle_load( msg );
          break;

        case SubId_SUB_MSG_PARAM_IO_SYNC:
          handle_sync( msg );
          break;

        default:
          LOG_ERROR( "Unhandled ParamIO subId: %d", msg.payload.header.subId );
          break;
      }
    }
  }


  void publishDataMessage( const Message::SysData &msg )
  {
    s_sys_data_queue.push( msg );
  }
}    // namespace Orbit::Serial
