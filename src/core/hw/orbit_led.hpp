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
  static constexpr uint8_t INVALID_POS    = 7;

#if defined( ORBIT_ESC_V3 )
  static constexpr uint8_t FAULT_POS      = 0;
  static constexpr uint8_t ARMED_POS      = 1;
  static constexpr uint8_t HEARTBEAT_POS  = 2;
  static constexpr uint8_t CAN_ACTIVE_POS = 3;
  static constexpr uint8_t USB_ACTIVE_POS = 4;
  static constexpr uint8_t STATUS_0_POS   = INVALID_POS;
  static constexpr uint8_t STATUS_1_POS   = INVALID_POS;
  static constexpr uint8_t STATUS_2_POS   = INVALID_POS;
  static constexpr uint8_t NUM_LEDS       = 5;
#elif defined( ORBIT_ESC_V2 )
  static constexpr uint8_t FAULT_POS      = 0;
  static constexpr uint8_t ARMED_POS      = 1;
  static constexpr uint8_t HEARTBEAT_POS  = 2;
  static constexpr uint8_t CAN_ACTIVE_POS = 3;
  static constexpr uint8_t STATUS_0_POS   = 4;
  static constexpr uint8_t STATUS_1_POS   = 5;
  static constexpr uint8_t STATUS_2_POS   = 6;
  static constexpr uint8_t USB_ACTIVE_POS = INVALID_POS;
  static constexpr uint8_t NUM_LEDS       = 7;
#elif defined( ORBIT_ESC_V1 )
  static constexpr uint8_t FAULT_POS      = 0;
  static constexpr uint8_t ARMED_POS      = 1;
  static constexpr uint8_t HEARTBEAT_POS  = 2;
  static constexpr uint8_t STATUS_0_POS   = 3;
  static constexpr uint8_t STATUS_1_POS   = 4;
  static constexpr uint8_t STATUS_2_POS   = 5;
  static constexpr uint8_t CAN_ACTIVE_POS = 6;
  static constexpr uint8_t USB_ACTIVE_POS = INVALID_POS;
  static constexpr uint8_t NUM_LEDS       = 7;
#endif

  static constexpr uint8_t FAULT_MSK      = 1u << FAULT_POS;
  static constexpr uint8_t ARMED_MSK      = 1u << ARMED_POS;
  static constexpr uint8_t HEARTBEAT_MSK  = 1u << HEARTBEAT_POS;
  static constexpr uint8_t STATUS_0_MSK   = 1u << STATUS_0_POS;
  static constexpr uint8_t STATUS_1_MSK   = 1u << STATUS_1_POS;
  static constexpr uint8_t STATUS_2_MSK   = 1u << STATUS_2_POS;
  static constexpr uint8_t USB_ACTIVE_MSK = 1u << USB_ACTIVE_POS;
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
    USB_ACTIVE = USB_ACTIVE_MSK,
    ALL        = ALL_LED_MSK,

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
  void clearChannel( const Channel channel );

  /**
   * @brief Toggles the state of the given channel
   *
   * @param channel   Which channel to toggle
   */
  void toggleChannel( const Channel channel );

  /**
   * @brief Process the state of active LEDs
   * @return void
   */
  void process();

  /**
   * @brief Registers callbacks with the USB subsystem to indicate when USB is active.
   *
   * This is a delayed operation that must be called after the USB driver has been
   * initialized.
   *
   * @return void
   */
  void attachUSBActiveListener();

}    // namespace Orbit::LED

#endif /* !ORBIT_LED_DRIVER_HPP */
