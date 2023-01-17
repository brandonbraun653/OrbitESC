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
#include <Aurora/logging>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/com/serial/serial_async_message.hpp>
#include <src/core/com/serial/serial_router.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::Serial
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  Chimera::Serial::Driver_rPtr serial;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {
    serial = Chimera::Serial::getDriver( IO::USART::serialChannel );
  }

  void processSerial()
  {
    uint8_t tmp[ 100 ];
    memset( tmp, 0, ARRAY_BYTES( tmp ) );
    size_t  read_size = serial->read( tmp, ARRAY_BYTES( tmp ) );
    if ( read_size > 0 )
    {
      serial->write( tmp, read_size );
    }
  }
}    // namespace Orbit::Serial
