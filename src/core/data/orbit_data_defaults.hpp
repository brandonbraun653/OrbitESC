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

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  System Configuration
  ---------------------------------------------------------------------------*/
  /**
   * @brief How often to synchronize the data storage disk
   */
  static constexpr size_t DFLT_DISK_SYNC_PERIOD_MS = 500;

  /*---------------------------------------------------------------------------
  System Identity
  ---------------------------------------------------------------------------*/
  static constexpr etl::string_view DFLT_BOARD_NAME = "OrbitESC";
  static constexpr etl::string_view DFLT_DESCRIPTION = "BLDC Motor Controller";

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
  Motor Power Stage
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
   * @brief Which commutation cycle to align with during PARK phase
   */
  static constexpr uint8_t DFLT_STATOR_ALIGN_COMM_PHASE = 1u;

  /*---------------------------------------------------------------------------
  Ramp Controller
  ---------------------------------------------------------------------------*/
  /**
   * @brief Target RPM to achieve when moving through the RAMP phase of startup
   */
  static constexpr float DFLT_RAMP_TARGET_RPM = 1'000.0f;

  /**
   * @brief Sets PWM duty cycle for the output stage during RAMP
   *
   * This controls the drive strength by changing how much time current is
   * allowed to ramp inside the stator windings. Higher percent duty cycles
   * yields more current => stronger EM fields => more torque.
   */
  static constexpr float DFLT_RAMP_DRIVE_STRENGTH_PCT = 25.0f;

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_DEFAULTS_HPP */
