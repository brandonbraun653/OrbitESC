/******************************************************************************
 *  File Name:
 *    can_message.hpp
 *
 *  Description:
 *    CAN bus message Definitions
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_MESSAGES_HPP
#define ORBIT_CAN_MESSAGES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/can_message_intf.hpp>
#include <etl/fsm.h>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum Id : etl::message_id_t
  {
    /*-------------------------------------------------------------------------
    Command and Control
    -------------------------------------------------------------------------*/
    MSG_PING            = 0x10, /**< Simple PING to see if the node is alive */
    MSG_SET_SYSTEM_MODE = 0x11, /**< Command a mode switch*/
    MSG_SET_MOTOR_SPEED = 0x12, /**< Set the motor speed */
    MSG_SET_CONFIG_DATA = 0x13, /**< Assignment of config data */
    MSG_GET_CONFIG_DATA = 0x14, /**< Request for config data from the board */
    MSG_RSP_CONFIG_DATA = 0x15, /**< Response to MSG_GET_CONFIG_DATA */
    MSG_EMERGENCY_HALT  = 0x16, /**< Command to safe-state the ESC */
    MSG_SYSTEM_RESET    = 0x17, /**< Fully reset/reboot the ESC */

    /*-------------------------------------------------------------------------
    Periodic Data
      Note: Don't forget to update numPeriodicMessageTypes()
    -------------------------------------------------------------------------*/
    MSG_PRDC_ADC_VDD             = 0x20,  /**< Power supply voltage */
    MSG_PRDC_ADC_PHASE_A_CURRENT = 0x21,  /**< Phase A current */
    MSG_PRDC_ADC_PHASE_B_CURRENT = 0x22,  /**< Phase B current */
    MSG_PRDC_ADC_PHASE_C_CURRENT = 0x23,  /**< Phase C current */
    MSG_PRDC_MOTOR_SPEED         = 0x24,  /**< Current motor speed */
    MSG_PRDC_SPEED_REF           = 0x25,  /**< Motor reference speed */
    MSG_PRDC_SYSTEM_TICK         = 0x50,  /**< Current system tick */
    MSG_PRDC_SYSTEM_MODE         = 0x51,  /**< Current system mode */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Returns the default frame for a CAN message
   *
   * @return Chimera::CAN::BasicFrame
   */
  static inline Chimera::CAN::BasicFrame defaultFrame()
  {
    Chimera::CAN::BasicFrame msg;
    msg.clear();
    msg.idMode    = Chimera::CAN::IdType::STANDARD;
    msg.frameType = Chimera::CAN::FrameType::DATA;

    return msg;
  }

}    // namespace Orbit::CAN::Message

#endif /* !ORBIT_CAN_MESSAGES_HPP */
