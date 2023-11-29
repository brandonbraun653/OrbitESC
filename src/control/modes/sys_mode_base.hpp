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
      EMERGENCY_HALT, /**< Emergency stop the motor and place into a safe state */
      ARM,            /**< Prepare the motor control system to run */
      DISARM,         /**< Move the motor controller to an idle state */
      ENGAGE,         /**< Engage the control system and drive the motor */
      DISENGAGE,      /**< Disengage the control system and stop driving the motor */
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
      IDLE,    /**< Control system is not engaged */
      ARMED,   /**< System is armed and ready to engage */
      ENGAGED, /**< Controller is running normally */
      FAULT,   /**< System is having problems */

      NUM_STATES
    };
  };

  /*---------------------------------------------------------------------------
  Event Message Types
  ---------------------------------------------------------------------------*/
  class MsgEmergencyHalt : public etl::message<EventId::EMERGENCY_HALT>
  {
  };

  class MsgArm : public etl::message<EventId::ARM>
  {
  };

  class MsgDisarm : public etl::message<EventId::DISARM>
  {
  };

  class MsgEngage : public etl::message<EventId::ENGAGE>
  {
  };

  class MsgDisengage : public etl::message<EventId::DISENGAGE>
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
