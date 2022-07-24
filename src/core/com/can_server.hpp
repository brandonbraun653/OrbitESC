/******************************************************************************
 *  File Name:
 *    can_server.hpp
 *
 *  Description:
 *    CAN bus dispatch communication server
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_DISPATCH_SERVER_HPP
#define ORBIT_CAN_DISPATCH_SERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>
#include <etl/message_bus.h>

namespace Orbit::CAN
{

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class DispatchServer : public etl::message_bus<8>
  {
  public:
    DispatchServer();
    ~DispatchServer();

    Chimera::Status_t initialize();

    void process();
  };
}    // namespace Orbit::CAN

#endif /* !ORBIT_CAN_DISPATCH_SERVER_HPP */
