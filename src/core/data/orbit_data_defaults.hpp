/******************************************************************************
 *  File Name:
 *    orbit_data_defaults.hpp
 *
 *  Description:
 *    Default data for controllable parameters
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_DEFAULTS_HPP
#define ORBIT_DATA_DEFAULTS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <etl/string.h>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/data/orbit_data_types.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  File System
  ---------------------------------------------------------------------------*/
  /**
   * @brief Dynamic mount point for the file system driver
   */
  static const etl::string_view FileSystemMountPoint = "/";

  /**
   * @brief Name of the file used internally to back our key-value store
   */
  static const etl::string_view SystemConfigFile = "/sys_config.bin";

  /**
   * @brief Name of the file used internally to store logs to
   */
  static const etl::string_view SystemLogFile = "/sys_log.txt";

  /*---------------------------------------------------------------------------
  System Configuration
  ---------------------------------------------------------------------------*/
  /**
   * @brief How often to synchronize the data storage disk
   */
  static constexpr size_t DFLT_DISK_SYNC_PERIOD_MS = 500;

  /**
   * @brief Scaling constant for the activity LED blink rate
   */
  static constexpr float DFLT_ACTIVITY_LED_SCALER = 1.0f;

  /**
   * @brief Default CAN node ID for the system
   */
  static constexpr Orbit::CAN::NodeId DFLT_CAN_NODE_ID = Orbit::CAN::NodeId::NODE_1;

  /*---------------------------------------------------------------------------
  System Identity
  ---------------------------------------------------------------------------*/
  static constexpr etl::string_view DFLT_BOARD_NAME       = "OrbitESC";
  static constexpr etl::string_view DFLT_DESCRIPTION      = "BLDC Motor Controller";
  static constexpr etl::string_view DFLT_SERIAL_NUMBER    = "xxxxxx";
  static constexpr etl::string_view DFLT_FIRMWARE_VERSION = "0.3.0";
  static constexpr uint8_t          DFLT_HARDWARE_VERSION = 2;
  static constexpr uint32_t         DFLT_DEVICE_ID        = 0x00000000;

  /*---------------------------------------------------------------------------
  Motor Characteristics
  ---------------------------------------------------------------------------*/
  /**
   * @brief Total number of poles on the rotor
   *
   * Each pole is a pair of magnetic fields (aka each rotor magnet). Not to be
   * confused with a stator "slot", which corresponds to the number of coils.
   */
  static constexpr size_t DFLT_ROTOR_NUM_POLES = 14;

  /**
   * @brief Total number of slots (windings) on the stator
   */
  static constexpr size_t DFLT_STATOR_NUM_SLOTS = 12;


  /*---------------------------------------------------------------------------
  Motor Control
  ---------------------------------------------------------------------------*/
  /**
   * @brief Core PWM frequency to switch the stator at in Hertz
   */
  static constexpr float DFLT_STATOR_PWM_FREQ_HZ = 10'000.0f;

  /**
   * @brief Core speed controller update frequency
   */
  static constexpr float DFLT_SPEED_CTL_UPDT_FREQ_HZ = 2'000.0f;

  /**
   * @brief Target RPM to achieve when idle
   */
  static constexpr float DFLT_TARGET_IDLE_RPM = 1'000.0f;

  /**
   * @brief Default FIR filter settings for the current controller
   */
  static constexpr float DFLT_ICTRL_DQ_FIR_FILTER[ Controls::FIRFilter::CoefData::SIZE ] = {
    -0.00124568841F, 0.00147019443F, 0.0123328818F,  0.00110139197F, -0.0499843247F, -0.0350326933F,
    0.164325342F,    0.407320708F,   0.407320708F,   0.164325342F,   -0.0350326933F, -0.0499843247F,
    0.00110139197F,  0.0123328818F,  0.00147019443F, -0.00124568841F
  };

  /**
   * @brief PID tunings for the Q/D axis in the inner loop current controller
   * https://www.mathworks.com/help/mcb/gs/sensorless-foc-pmsm-smo-fo.html
   */
  static constexpr float DFLT_ICTRL_Q_PID_KP = 1.7011f;
  static constexpr float DFLT_ICTRL_Q_PID_KI = 0.15494f;
  static constexpr float DFLT_ICTRL_Q_PID_KD = 0.0f;
  static constexpr float DFLT_ICTRL_D_PID_KP = 1.7011f;
  static constexpr float DFLT_ICTRL_D_PID_KI = 0.15494f;
  static constexpr float DFLT_ICTRL_D_PID_KD = 0.0f;

  /**
   * @brief Current observer tuning parameters
   */
  static constexpr float DFLT_CURRENT_OBSERVER_KSLIDE    = 3.5f;
  static constexpr float DFLT_CURRENT_OBSERVER_MAX_ERROR = 0.1f;

  /**
   * @brief Ramp controller acceleration curve constants
   */
  static constexpr float DFLT_RAMP_CTRL_FIRST_ORDER_TERM  = 1.5f;
  static constexpr float DFLT_RAMP_CTRL_SECOND_ORDER_TERM = 20.0f;
  static constexpr float DFLT_RAMP_CTRL_RAMP_TIME_SEC     = 1.35f;

  /**
   * @brief PID tunings for the speed controller
   */
  static constexpr float DFLT_SPEED_PID_KP = 0.1f;
  static constexpr float DFLT_SPEED_PID_KI = 0.0f;
  static constexpr float DFLT_SPEED_PID_KD = 0.0f;

  /**
   * @brief Scale the maximum drive strength of the motor drive output.
   */
  static constexpr float DFLT_PWM_DRIVE_MAX_DUTY = 0.95f;

  /*---------------------------------------------------------------------------
  Motor Description
  ---------------------------------------------------------------------------*/
  static constexpr float DFLT_STATOR_INDUCTANCE  = 380.0f * 1e-3f;
  static constexpr float DFLT_STATOR_RESISTANCE  = 0.01f;
  static constexpr float DFLT_PEAK_PHASE_CURRENT = 10.0f;
  static constexpr float DFLT_PEAK_VOLTAGE       = 16.0f;

  /*---------------------------------------------------------------------------
  Engagment Thresholds
  ---------------------------------------------------------------------------*/
  static constexpr float DFLT_MIN_ARM_VOLTAGE = 8.0f;
  static constexpr float DFLT_MAX_ARM_VOLTAGE = 18.0f;

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_DEFAULTS_HPP */
