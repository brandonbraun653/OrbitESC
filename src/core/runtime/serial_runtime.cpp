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
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_router.hpp>
#include <src/core/com/serial/serial_server.hpp>
#include <src/core/data/orbit_data_storage.hpp>
#include <src/core/runtime/serial_runtime.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static etl::circular_buffer<uint8_t, 1024> s_msg_buffer;
  static Orbit::Serial::DispatchServer       s_server;

  /*---------------------------------------------------------------------------
  Router Declarations
  ---------------------------------------------------------------------------*/
  static Router::PingRouter    s_ping_router;
  static Router::ParamIORouter s_param_router;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Handle requests to GET a parameter value
   * @param msg Message request
   */
  static void handle_get( const Message::ParamIO &msg )
  {
  }


  /**
   * @brief Handle requests to PUT a parameter value
   * @param msg Message request
   */
  static void handle_put( const Message::ParamIO &msg )
  {
  }


  /**
   * @brief Handle requests to LOAD data from disk
   * @param msg Message request
   */
  static void handle_load( const Message::ParamIO &msg )
  {
    sendAckNack( Data::loadDisk(), msg.payload.header );
  }


  /**
   * @brief Handle requests to SYNC changes to disk
   * @param msg Message request
   */
  static void handle_sync( const Message::ParamIO &msg )
  {
    sendAckNack( Data::flushDisk(), msg.payload.header );
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {
    /*-------------------------------------------------------------------------
    Initialize the core server
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_server.initialize( IO::USART::serialChannel, s_msg_buffer ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Register the routers to handle incoming messages
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_server.subscribe( s_ping_router ) );
    RT_HARD_ASSERT( s_server.subscribe( s_param_router ) );
  }


  void processSerial()
  {
    s_server.process();
  }


  void handleParamIOEvent()
  {
    while ( Router::ParamIOEventQueue.size() )
    {
      Message::ParamIO msg = Router::ParamIOEventQueue.front();
      Router::ParamIOEventQueue.pop();

      switch ( msg.payload.header.subId )
      {
        case Message::SUB_MSG_PARAM_IO_GET:
          handle_get( msg );
          break;

        case Message::SUB_MSG_PARAM_IO_PUT:
          handle_put( msg );
          break;

        case Message::SUB_MSG_PARAM_IO_LOAD:
          handle_load( msg );
          break;

        case Message::SUB_MSG_PARAM_IO_SYNC:
          handle_sync( msg );
          break;

        default:
          LOG_ERROR( "Unhandled ParamIO subId: %d", msg.payload.header.subId );
          break;
      }
    }
  }
}    // namespace Orbit::Serial
