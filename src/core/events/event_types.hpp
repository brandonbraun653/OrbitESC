/******************************************************************************
 *  File Name:
 *    event_types.hpp
 *
 *  Description:
 *    Various system event messages and tyeps
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_EVENT_TYPES_HPP
#define ORBIT_ESC_EVENT_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/message.h>
#include <src/core/com/proto/serial_interface.pb.h>


namespace Orbit::Event
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Alias the definitions from the protocol buffer interface spec
   */
  enum _EventID : etl::message_id_t
  {
    EVENT_SYS_RESET,
    EVENT_CFG_PHASE_CURRENT_STREAM,
    EVENT_CFG_SYSTEM_VOLTAGE_STREAM,
    EVENT_CFG_PWM_COMMAND_STREAM,
    EVENT_CFG_STATE_ESTIMATE_STREAM,

    EVENT_COUNT
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct SystemReset : public etl::message<EVENT_SYS_RESET>
  {
  };

  struct StreamPhaseCurrents : public etl::message<EVENT_CFG_PHASE_CURRENT_STREAM>
  {
    bool enable;
  };

  struct StreamSystemVoltages : public etl::message<EVENT_CFG_SYSTEM_VOLTAGE_STREAM>
  {
    bool enable;
  };

  struct StreamPWMCommands : public etl::message<EVENT_CFG_PWM_COMMAND_STREAM>
  {
    bool enable;
  };

  struct StreamStateEstimates : public etl::message<EVENT_CFG_STATE_ESTIMATE_STREAM>
  {
    bool enable;
  };

}    // namespace Orbit::Event

#endif /* !ORBIT_ESC_EVENT_TYPES_HPP */
