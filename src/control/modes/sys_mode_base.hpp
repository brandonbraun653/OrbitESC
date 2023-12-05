/******************************************************************************
 *  File Name:
 *    sys_mode_base.hpp
 *
 *  Description:
 *    System mode controller
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SYSTEM_MODE_BASE_HPP
#define ORBIT_ESC_SYSTEM_MODE_BASE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <etl/fsm.h>
#include <etl/message.h>
#include <etl/string_view.h>
#include <limits>
#include <src/control/foc_data.hpp>
#include <src/core/com/proto/motor_control.pb.h>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr etl::message_router_id_t MOTOR_STATE_ROUTER_ID = std::numeric_limits<etl::message_router_id_t>::max() - 1;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Events that can cause a state transition
   */
  using EventId_t = etl::message_id_t;
  struct EventId
  {
    enum
    {
      ARM,            /**< Prepare the motor control system to run */
      ENGAGE,         /**< Engage the control system and drive the motor */
      DISABLE,        /**< Move the motor controller to an idle state */
      FAULT,          /**< Indicate a fault has occurred */

      NUM_EVENTS
    };
  };

  /**
   * @brief States the motor control system can be in
   */
  using ModeId_t = etl::fsm_state_id_t;
  struct ModeId
  {
    enum
    {
      IDLE    = MotorCtrlState_MOTOR_CTRL_STATE_IDLE,    /**< Control system is not engaged */
      ARMED   = MotorCtrlState_MOTOR_CTRL_STATE_ARMED,   /**< System is armed and ready to engage */
      ENGAGED = MotorCtrlState_MOTOR_CTRL_STATE_ENGAGED, /**< Controller is running normally */
      FAULT   = MotorCtrlState_MOTOR_CTRL_STATE_FAULT,   /**< System is having problems */

      NUM_STATES = 4
    };
  };

  /*---------------------------------------------------------------------------
  Event Message Types
  ---------------------------------------------------------------------------*/

  class MsgArm : public etl::message<EventId::ARM>
  {
  };

  class MsgEngage : public etl::message<EventId::ENGAGE>
  {
  };

  class MsgDisable : public etl::message<EventId::DISABLE>
  {
  };

  class MsgFault : public etl::message<EventId::FAULT>
  {
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Gets a stringified name of the event
   *
   * @param event   Which event to get the name of
   * @return const etl::string_view&
   */
  const etl::string_view &getMessageString( const EventId_t event );

  /**
   * @brief Gets a stringified name of the mode
   *
   * @param state   Which mode to look up
   * @return const etl::string_view&
   */
  const etl::string_view &getModeString( const ModeId_t mode );

}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_SYSTEM_MODE_BASE_HPP */
