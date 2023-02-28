/******************************************************************************
 *  File Name:
 *    can_periodic_message.hpp
 *
 *  Description:
 *    Periodic CAN bus message descriptors
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CAN_PERIODIC_MESSAGE_HPP
#define ORBIT_CAN_PERIODIC_MESSAGE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/can/can_message.hpp>
#include <src/core/com/can/can_message_intf.hpp>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  Periodic Messages: See can_message.hpp for descriptions
  ---------------------------------------------------------------------------*/
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


  class PhaseCCurrent : public Attributes<PhaseCCurrent, MSG_PRDC_ADC_PHASE_C_CURRENT, 100>
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


  class SpeedReference : public Attributes<SpeedReference, MSG_PRDC_SPEED_REF, 100>
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
  template<class... T>
  static constexpr size_t calcPeriodicMessageTypes()
  {
    return ( 0 + ... + T::_foldExprPeriodicCounter() );
  }

  static constexpr size_t numPeriodicMessageTypes()
  {
    return calcPeriodicMessageTypes<SystemTick, SystemMode, PowerSupplyVoltage, PhaseACurrent, PhaseBCurrent, PhaseCCurrent,
                                    MotorSpeed>();
  }
}  // namespace Orbit::CAN::Message

#endif  /* !ORBIT_CAN_PERIODIC_MESSAGE_HPP */
