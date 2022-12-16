/******************************************************************************
 *  File Name:
 *    orbit_led.hpp
 *
 *  Description:
 *    LED driver interface for Orbit ESC
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_LED_DRIVER_HPP
#define ORBIT_LED_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::LED
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
#if defined( ORBIT_ESC_V2 )
  static constexpr uint8_t FAULT_POS      = 0;
  static constexpr uint8_t ARMED_POS      = 1;
  static constexpr uint8_t HEARTBEAT_POS  = 2;
  static constexpr uint8_t CAN_ACTIVE_POS = 3;
  static constexpr uint8_t STATUS_0_POS   = 4;
  static constexpr uint8_t STATUS_1_POS   = 5;
  static constexpr uint8_t STATUS_2_POS   = 6;
#elif defined( ORBIT_ESC_V1 )
  static constexpr uint8_t FAULT_POS      = 0;
  static constexpr uint8_t ARMED_POS      = 1;
  static constexpr uint8_t HEARTBEAT_POS  = 2;
  static constexpr uint8_t STATUS_0_POS   = 3;
  static constexpr uint8_t STATUS_1_POS   = 4;
  static constexpr uint8_t STATUS_2_POS   = 5;
  static constexpr uint8_t CAN_ACTIVE_POS = 6;
#endif
  static constexpr uint8_t INVALID_POS    = 7;

  static constexpr uint8_t FAULT_MSK      = 1u << FAULT_POS;
  static constexpr uint8_t ARMED_MSK      = 1u << ARMED_POS;
  static constexpr uint8_t HEARTBEAT_MSK  = 1u << HEARTBEAT_POS;
  static constexpr uint8_t STATUS_0_MSK   = 1u << STATUS_0_POS;
  static constexpr uint8_t STATUS_1_MSK   = 1u << STATUS_1_POS;
  static constexpr uint8_t STATUS_2_MSK   = 1u << STATUS_2_POS;
  static constexpr uint8_t CAN_ACTIVE_MSK = 1u << CAN_ACTIVE_POS;
  static constexpr uint8_t INVALID_MSK    = 1u << INVALID_POS;
  static constexpr uint8_t ALL_LED_MSK    = 0x7F;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum Channel : uint8_t
  {
    FAULT      = FAULT_MSK,
    ARMED      = ARMED_MSK,
    HEARTBEAT  = HEARTBEAT_MSK,
    STATUS_O   = STATUS_0_MSK,
    STATUS_1   = STATUS_1_MSK,
    STATUS_2   = STATUS_2_MSK,
    CAN_ACTIVE = CAN_ACTIVE_MSK,

    NUM_OPTIONS = INVALID_MSK,
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers on the module
   * @note This depends on SPI having already been configured
   */
  void powerUp();

  /**
   * @brief Updates the LED driver output with the latest state
   */
  void sendUpdate();

  /**
   * @brief Enables multiple LEDs
   *
   * @param mask  Enables all LEDs with bits set
   */
  void setBits( const uint8_t mask );

  /**
   * @brief Disables multiple LEDs
   *
   * @param mask  Disables all LEDs with bits set
   */
  void clrBits( const uint8_t mask );

  /**
   * @brief Get the current LED bit mask being presented to the user
   * @return uint8_t
   */
  uint8_t getBits();

  /**
   * @brief Enables an individual LED
   *
   * @param channel   Which LED to enable
   */
  void setChannel( const Channel channel );

  /**
   * @brief Disables an individual LED
   *
   * @param channel   Which LED to enable
   */
  void clrChannel( const Channel channel );

  /**
   * @brief Toggles the state of the given channel
   *
   * @param channel   Which channel to toggle
   */
  void toggleChannel( const Channel channel );

}    // namespace Orbit::LED

#endif /* !ORBIT_LED_DRIVER_HPP */
