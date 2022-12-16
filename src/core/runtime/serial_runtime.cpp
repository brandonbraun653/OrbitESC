/******************************************************************************
 *  File Name:
 *    serial_runtime.cpp
 *
 *  Description:
 *    Serial bus processing
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/com/serial/serial_message.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_router.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {

  }

  void processSerial()
  {
    auto msg = Message::Ping();

    msg.send( Chimera::Serial::getDriver( IO::USART::serialChannel ) );
  }
}  // namespace Orbit::Serial
