/******************************************************************************
 *  File Name:
 *    orbit_data_params.hpp
 *
 *  Description:
 *    Parameter listings for the whole orbit data system
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_PARAMETERS_HPP
#define ORBIT_DATA_PARAMETERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <array>
#include <algorithm>
#include <cstddef>
#include <src/core/com/serial/serial_interface.pb.h>
#include <src/core/data/orbit_data_validators.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using ParameterList = std::array<ParameterNode, 37>;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr const char *FMT_UINT8  = "%u";
  static constexpr const char *FMT_UINT16 = FMT_UINT8;
  static constexpr const char *FMT_UINT32 = "%lu";
  static constexpr const char *FMT_FLOAT  = "%4.9f";
  static constexpr const char *FMT_DOUBLE = "%4.17f";
  static constexpr const char *FMT_STRING = "%s";
  static constexpr const char *FMT_BOOL   = "%d";

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  namespace Internal
  {
    static constexpr ParameterList _compile_time_sort( const ParameterList &list )
    {
      auto result = list;
      std::sort( result.begin(), result.end(),
                 []( const ParameterNode &a, const ParameterNode &b ) -> bool { return a.id < b.id; } );
      return result;
    }

    static constexpr ParameterList _unsorted_parameters = {
      /*-----------------------------------------------------------------------------
      Read Only Parameters
      -----------------------------------------------------------------------------*/
      ParameterNode{ .id        = ParamId_PARAM_BOOT_COUNT,
                     .type      = ParamType_UINT32,
                     .key       = "pwr_cnt",
                     .address   = &SysInfo.bootCount,
                     .maxSize   = sizeof( SysInfo.bootCount ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_HW_VERSION,
                     .type      = ParamType_UINT8,
                     .key       = "hw_ver",
                     .address   = &SysIdentity.hwVersion,
                     .maxSize   = sizeof( SysIdentity.hwVersion ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_SW_VERSION,
                     .type      = ParamType_STRING,
                     .key       = "sw_ver",
                     .address   = &SysIdentity.swVersion,
                     .maxSize   = SysIdentity.swVersion.MAX_SIZE,
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_DEVICE_ID,
                     .type      = ParamType_UINT32,
                     .key       = "dev_id",
                     .address   = &SysIdentity.deviceId,
                     .maxSize   = sizeof( SysIdentity.deviceId ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_BOARD_NAME,
                     .type      = ParamType_STRING,
                     .key       = "name",
                     .address   = &SysIdentity.boardName,
                     .maxSize   = SysIdentity.boardName.MAX_SIZE,
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_DESCRIPTION,
                     .type      = ParamType_STRING,
                     .key       = "desc",
                     .address   = &SysIdentity.description,
                     .maxSize   = SysIdentity.description.MAX_SIZE,
                     .validator = nullptr },

      /*-----------------------------------------------------------------------------
      Read/Write Parameters
      -----------------------------------------------------------------------------*/
      ParameterNode{ .id        = ParamId_PARAM_SERIAL_NUMBER,
                     .type      = ParamType_STRING,
                     .key       = "ser_num",
                     .address   = &SysIdentity.serialNumber,
                     .maxSize   = SysIdentity.serialNumber.MAX_SIZE,
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_DISK_UPDATE_RATE_MS,
                     .type      = ParamType_UINT32,
                     .key       = "dsk_updt",
                     .address   = &SysConfig.diskUpdateRateMs,
                     .maxSize   = sizeof( SysConfig.diskUpdateRateMs ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_ACTIVITY_LED_SCALER,
                     .type      = ParamType_FLOAT,
                     .key       = "actv_led_scale",
                     .address   = &SysConfig.activityLedScaler,
                     .maxSize   = sizeof( SysConfig.activityLedScaler ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_BOOT_MODE,
                     .type      = ParamType_UINT8,
                     .key       = "boot_mode",
                     .address   = &SysInfo.bootMode,
                     .maxSize   = sizeof( SysInfo.bootMode ),
                     .validator = nullptr },

      ParameterNode{ .id        = ParamId_PARAM_CAN_NODE_ID,
                     .type      = ParamType_UINT8,
                     .key       = "can_id",
                     .address   = &SysConfig.canNodeId,
                     .maxSize   = sizeof( SysConfig.canNodeId ),
                     .validator = ValidateParamId_PARAM_CAN_NODE_ID },

      /*-----------------------------------------------------------------------
      Motor Control Parameters
      -----------------------------------------------------------------------*/
      ParameterNode { .id       = ParamId_PARAM_STATOR_PWM_FREQ,
                      .type     = ParamType_FLOAT,
                      .key      = "stator_pwm_freq",
                      .address  = &SysControl.statorPWMFreq,
                      .maxSize  = sizeof( SysControl.statorPWMFreq ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_SPEED_CTRL_FREQ,
                      .type     = ParamType_FLOAT,
                      .key      = "spd_ctrl_freq",
                      .address  = &SysControl.speedCtrlUpdateFreq,
                      .maxSize  = sizeof( SysControl.speedCtrlUpdateFreq ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_SPEED_CTRL_KP,
                      .type     = ParamType_FLOAT,
                      .key      = "spd_ctrl_kp",
                      .address  = &SysControl.speedCtrlKp,
                      .maxSize  = sizeof( SysControl.speedCtrlKp ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_SPEED_CTRL_KI,
                      .type     = ParamType_FLOAT,
                      .key      = "spd_ctrl_ki",
                      .address  = &SysControl.speedCtrlKi,
                      .maxSize  = sizeof( SysControl.speedCtrlKi ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_SPEED_CTRL_KD,
                      .type     = ParamType_FLOAT,
                      .key      = "spd_ctrl_kd",
                      .address  = &SysControl.speedCtrlKd,
                      .maxSize  = sizeof( SysControl.speedCtrlKd ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_TARGET_IDLE_RPM,
                      .type     = ParamType_FLOAT,
                      .key      = "target_idle_rpm",
                      .address  = &SysControl.targetIdleRPM,
                      .maxSize  = sizeof( SysControl.targetIdleRPM ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KP,
                      .type     = ParamType_FLOAT,
                      .key      = "iq_ctrl_kp",
                      .address  = &SysControl.currentCtrl_Q_Kp,
                      .maxSize  = sizeof( SysControl.currentCtrl_Q_Kp ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KI,
                      .type     = ParamType_FLOAT,
                      .key      = "iq_ctrl_ki",
                      .address  = &SysControl.currentCtrl_Q_Ki,
                      .maxSize  = sizeof( SysControl.currentCtrl_Q_Ki ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KD,
                      .type     = ParamType_FLOAT,
                      .key      = "iq_ctrl_kd",
                      .address  = &SysControl.currentCtrl_Q_Kd,
                      .maxSize  = sizeof( SysControl.currentCtrl_Q_Kd ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_CTRL_D_AXIS_KP,
                      .type     = ParamType_FLOAT,
                      .key      = "id_ctrl_kp",
                      .address  = &SysControl.currentCtrl_D_Kp,
                      .maxSize  = sizeof( SysControl.currentCtrl_D_Kp ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_CTRL_D_AXIS_KI,
                      .type     = ParamType_FLOAT,
                      .key      = "id_ctrl_ki",
                      .address  = &SysControl.currentCtrl_D_Ki,
                      .maxSize  = sizeof( SysControl.currentCtrl_D_Ki ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_CTRL_D_AXIS_KD,
                      .type     = ParamType_FLOAT,
                      .key      = "id_ctrl_kd",
                      .address  = &SysControl.currentCtrl_D_Kd,
                      .maxSize  = sizeof( SysControl.currentCtrl_D_Kd ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_RAMP_CTRL_FIRST_ORDER_TERM,
                      .type     = ParamType_FLOAT,
                      .key      = "ramp_1st_term",
                      .address  = &SysControl.rampCtrlFirstOrderTerm,
                      .maxSize  = sizeof( SysControl.rampCtrlFirstOrderTerm ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_RAMP_CTRL_SECOND_ORDER_TERM,
                      .type     = ParamType_FLOAT,
                      .key      = "ramp_2nd_term",
                      .address  = &SysControl.rampCtrlSecondOrderTerm,
                      .maxSize  = sizeof( SysControl.rampCtrlSecondOrderTerm ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_RAMP_CTRL_RAMP_TIME_SEC,
                      .type     = ParamType_FLOAT,
                      .key      = "ramp_time_sec",
                      .address  = &SysControl.rampCtrlRampTimeSec,
                      .maxSize  = sizeof( SysControl.rampCtrlRampTimeSec ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_OBSERVER_KSLIDE,
                      .type     = ParamType_FLOAT,
                      .key      = "smc_kslide",
                      .address  = &SysControl.currentObserver_KSlide,
                      .maxSize  = sizeof( SysControl.currentObserver_KSlide ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_CURRENT_OBSERVER_MAX_ERROR,
                      .type     = ParamType_FLOAT,
                      .key      = "smc_max_error",
                      .address  = &SysControl.currentObserver_MaxError,
                      .maxSize  = sizeof( SysControl.currentObserver_MaxError ),
                      .validator = nullptr },

      /*-----------------------------------------------------------------------
      Motor Description Parameters
      -----------------------------------------------------------------------*/
      ParameterNode { .id       = ParamId_PARAM_ROTOR_POLES,
                      .type     = ParamType_UINT8,
                      .key      = "rotor_poles",
                      .address  = &SysConfig.rotorPoles,
                      .maxSize  = sizeof( SysConfig.rotorPoles ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_STATOR_SLOTS,
                      .type     = ParamType_UINT8,
                      .key      = "stator_slots",
                      .address  = &SysConfig.statorSlots,
                      .maxSize  = sizeof( SysConfig.statorSlots ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_STATOR_RESISTANCE,
                      .type     = ParamType_FLOAT,
                      .key      = "stator_resistance",
                      .address  = &SysConfig.statorResistance,
                      .maxSize  = sizeof( SysConfig.statorResistance ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_STATOR_INDUCTANCE,
                      .type     = ParamType_FLOAT,
                      .key      = "stator_inductance",
                      .address  = &SysConfig.statorInductance,
                      .maxSize  = sizeof( SysConfig.statorInductance ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_PEAK_CURRENT_THRESHOLD,
                      .type     = ParamType_FLOAT,
                      .key      = "peak_current_threshold",
                      .address  = &SysConfig.peakCurrentThreshold,
                      .maxSize  = sizeof( SysConfig.peakCurrentThreshold ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_PEAK_VOLTAGE_THRESHOLD,
                      .type     = ParamType_FLOAT,
                      .key      = "peak_voltage_threshold",
                      .address  = &SysConfig.peakVoltageThreshold,
                      .maxSize  = sizeof( SysConfig.peakVoltageThreshold ),
                      .validator = nullptr },

      /*-----------------------------------------------------------------------
      System Configuration Parameters
      -----------------------------------------------------------------------*/
      ParameterNode { .id       = ParamId_PARAM_STREAM_PHASE_CURRENTS,
                      .type     = ParamType_BOOL,
                      .key      = "stream_phase_currents",
                      .address  = &SysConfig.streamPhaseCurrents,
                      .maxSize  = sizeof( SysConfig.streamPhaseCurrents ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_STREAM_PWM_COMMANDS,
                      .type     = ParamType_BOOL,
                      .key      = "stream_pwm_commands",
                      .address  = &SysConfig.streamPwmCommands,
                      .maxSize  = sizeof( SysConfig.streamPwmCommands ),
                      .validator = nullptr },

      ParameterNode { .id       = ParamId_PARAM_STREAM_STATE_ESTIMATES,
                      .type     = ParamType_BOOL,
                      .key      = "stream_state_estimates",
                      .address  = &SysConfig.streamStateEstimates,
                      .maxSize  = sizeof( SysConfig.streamStateEstimates ),
                      .validator = nullptr },

      /***** Add new entries above here *****/
    };
  }    // namespace Internal

}    // namespace Orbit::Data

#endif /* !ORBIT_DATA_PARAMETERS_HPP */
