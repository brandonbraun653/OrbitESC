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
#include <src/core/runtime/serial_runtime.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static etl::circular_buffer<uint8_t, 1024> s_msg_buffer;
  static Orbit::Serial::DispatchServer      s_server;

  /*---------------------------------------------------------------------------
  Router Declarations
  ---------------------------------------------------------------------------*/
  static Router::PingRouter s_ping_router;

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
  }

  void processSerial()
  {
    s_server.process();
  }
}    // namespace Orbit::Serial
