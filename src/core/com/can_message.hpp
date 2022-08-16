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
    MSG_PING            = 0x10,
    MSG_SET_SYSTEM_MODE = 0x11,
    MSG_SET_MOTOR_SPEED = 0x12,

    /*-------------------------------------------------------------------------
    Periodic Data
      Note: Don't forget to update numPeriodicMessageTypes()
    -------------------------------------------------------------------------*/
    MSG_PRDC_ADC_VDD             = 0x20,
    MSG_PRDC_ADC_PHASE_A_CURRENT = 0x21,
    MSG_PRDC_ADC_PHASE_B_CURRENT = 0x22,
    MSG_PRDC_MOTOR_SPEED         = 0x23,
    MSG_PRDC_SYSTEM_TICK         = 0x50,
    MSG_PRDC_SYSTEM_MODE         = 0x51,

    /*-------------------------------------------------------------------------
    Miscellaneous Messages
    -------------------------------------------------------------------------*/
    MSG_STREAM_BOOKEND = 0x60, /**< Start/stop of stream messages */
    MSG_STREAM_DATA    = 0x61, /**< Data payload for a stream */
  };


  /*---------------------------------------------------------------------------
  Asynchronous Messages
  ---------------------------------------------------------------------------*/
  /**
   * @brief Ping message to test the connection
   */
  class Ping : public Attributes<Ping, MSG_PING>
  {
  public:
    __packed_struct Payload
    {
      Header dst; /**< Destination node */
      Header src; /**< Source node */
    }
    payload;
  };


  /**
   * @brief Arm/Disarm message to control the motor
   */
  class SetSystemMode : public Attributes<SetSystemMode, MSG_SET_SYSTEM_MODE>
  {
  public:
    __packed_struct Payload
    {
      Header              dst;  /**< Destination node */
      etl::fsm_state_id_t mode; /**< New mode to switch into Orbit::Control::ModeId */
    }
    payload;
  };


  /**
   * @brief Set the motor speed reference
   */
  class SetMotorSpeed : public Attributes<SetMotorSpeed, MSG_SET_MOTOR_SPEED>
  {
  public:
    __packed_struct Payload
    {
      Header   dst;   /**< Destination node */
      uint16_t speed; /**< Motor speed in RPM */
    }
    payload;
  };


  class StreamBookend : public Attributes<StreamBookend, MSG_STREAM_BOOKEND>
  {
  public:
    __packed_struct Payload
    {
      Header   src;   /**< Source node */
      uint8_t  type;  /**< Type of stream */
      uint16_t bytes; /**< Number of bytes in the stream */
    }
    payload;
  };


  class StreamData : public Attributes<StreamData, MSG_STREAM_DATA>
  {
  public:
    __packed_struct Payload
    {
      Header  src;       /**< Source node */
      uint8_t type;      /**< Type of stream */
      uint8_t frame;     /**< Frame number of the stream */
      uint8_t data[ 5 ]; /**< Variable data amount  */
    }
    payload;
  };

  /*---------------------------------------------------------------------------
  Periodic TX Messages
  ---------------------------------------------------------------------------*/
  /**
   * @brief System time of the ESC node
   */
  class SystemTick : public Attributes<SystemTick, MSG_PRDC_SYSTEM_TICK, 1000>
  {
  public:
    __packed_struct Payload
    {
      Header   hdr;  /**< Message Header */
      uint32_t tick; /**< Current tick of system in milliseconds */
    }
    payload;

    void update() final override;
  };


  /**
   * @brief System mode of the ESC node
   */
  class SystemMode : public Attributes<SystemMode, MSG_PRDC_SYSTEM_MODE, 250>
  {
  public:
    __packed_struct Payload
    {
      Header              hdr;  /**< Message Header */
      etl::fsm_state_id_t mode; /**< Current system mode Orbit::Control::ModeId */
    }
    payload;

    void update() final override;
  };


  /**
   * @brief ADC power supply voltage reading
   */
  class PowerSupplyVoltage : public Attributes<PowerSupplyVoltage, MSG_PRDC_ADC_VDD, 100>
  {
  public:
    __packed_struct Payload
    {
      Header   hdr;       /**< Message Header */
      uint32_t timestamp; /**< Timestamp of the ADC reading in microseconds */
      uint16_t vdd;       /**< Voltage in millivolts */
    }
    payload;

    void update() final override;
  };


  /**
   * @brief ADC phase A current reading
   */
  class PhaseACurrent : public Attributes<PhaseACurrent, MSG_PRDC_ADC_PHASE_A_CURRENT, 100>
  {
  public:
    __packed_struct Payload
    {
      Header   hdr;       /**< Message Header */
      uint32_t timestamp; /**< Timestamp of the ADC reading in microseconds */
      uint16_t current;   /**< Current in milliamps */
    }
    payload;

    void update() final override;
  };


  /**
   * @brief ADC phase B current reading
   */
  class PhaseBCurrent : public Attributes<PhaseBCurrent, MSG_PRDC_ADC_PHASE_B_CURRENT, 100>
  {
  public:
    __packed_struct Payload
    {
      Header   hdr;       /**< Message Header */
      uint32_t timestamp; /**< Timestamp of the ADC reading in microseconds */
      uint16_t current;   /**< Current in milliamps */
    }
    payload;

    void update() final override;
  };


  /**
   * @brief Estimated motor speed in RPM
   */
  class MotorSpeed : public Attributes<MotorSpeed, MSG_PRDC_MOTOR_SPEED, 100>
  {
  public:
    __packed_struct Payload
    {
      Header   hdr;   /**< Message header */
      uint32_t tick;  /**< System time in milliseconds */
      uint16_t speed; /**< Motor speed in RPM */
    }
    payload;

    void update() final override;
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


  static constexpr size_t numPeriodicMessageTypes()
  {
    return calcPeriodicMessageTypes<SystemTick, SystemMode, PowerSupplyVoltage, PhaseACurrent, PhaseBCurrent, MotorSpeed>();
  }

}    // namespace Orbit::CAN::Message

#endif /* !ORBIT_CAN_MESSAGES_HPP */
