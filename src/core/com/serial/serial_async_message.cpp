/******************************************************************************
 *  File Name:
 *    serial_async_message.cpp
 *
 *  Description:
 *    Utility functions for serial messages
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/atomic.h>
#include <src/core/com/serial/serial_async_message.hpp>

namespace Orbit::Serial::Message
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile etl::atomic<UniqueId_t> s_msg_uuid;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void moduleInit()
  {
    s_msg_uuid = 0;
  }


  UniqueId_t getNextUUID()
  {
    return s_msg_uuid.fetch_add( 1, etl::memory_order_seq_cst );
  }
}  // namespace Orbit::Serial::Message
