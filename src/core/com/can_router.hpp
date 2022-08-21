/******************************************************************************
 *  File Name:
 *    can_router.hpp
 *
 *  Description:
 *    Routers for various CAN RX messages
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_ROUTERS_HPP
#define ORBIT_CAN_ROUTERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/can_message.hpp>
#include <src/core/com/can_async_message.hpp>
#include <etl/message_router.h>

namespace Orbit::CAN::Router
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class PingRouter : public etl::message_router<PingRouter, Message::Ping>
  {
  public:
    PingRouter();
    void on_receive( const Message::Ping &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

  class SetSystemModeRouter : public etl::message_router<SetSystemModeRouter, Message::SetSystemMode>
  {
  public:
    SetSystemModeRouter();
    void on_receive( const Message::SetSystemMode &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

  class SetMotorSpeedRouter : public etl::message_router<SetMotorSpeedRouter, Message::SetMotorSpeed>
  {
  public:
    SetMotorSpeedRouter();
    void on_receive( const Message::SetMotorSpeed &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

  class EmergencyHaltRouter : public etl::message_router<EmergencyHaltRouter, Message::EmergencyHalt>
  {
  public:
    EmergencyHaltRouter();
    void on_receive( const Message::EmergencyHalt &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

  class SystemResetRouter : public etl::message_router<SystemResetRouter, Message::SystemReset>
  {
  public:
    SystemResetRouter();
    void on_receive( const Message::SystemReset &msg );
    void on_receive_unknown( const etl::imessage &msg );
  };

}    // namespace Orbit::CAN::Router

#endif /* !ORBIT_CAN_ROUTERS_HPP */
