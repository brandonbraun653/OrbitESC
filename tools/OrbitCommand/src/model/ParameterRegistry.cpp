#include "model/ParameterRegistry.h"

namespace orbit::model::registry
{
  namespace
  {
    const QVector<ParameterDescriptor> &descriptors()
    {
      static const QVector<ParameterDescriptor> list = {
        { ParamId_PARAM_BOOT_COUNT, "pwr_cnt", "Boot Count", ParameterGroup::Information, ParamType_UINT32, true, {}, 0, 0, 4294967295.0, 32 },
        { ParamId_PARAM_HW_VERSION, "hw_ver", "Hardware Version", ParameterGroup::Information, ParamType_UINT8, true, {}, 0, 0, 255, 32 },
        { ParamId_PARAM_SW_VERSION, "sw_ver", "Software Version", ParameterGroup::Information, ParamType_STRING, true, {}, 0, 0, 0, 8 },
        { ParamId_PARAM_DEVICE_ID, "dev_id", "Device ID", ParameterGroup::Information, ParamType_UINT32, true, {}, 0, 0, 4294967295.0, 32 },
        { ParamId_PARAM_BOARD_NAME, "name", "Board Name", ParameterGroup::Information, ParamType_STRING, true, {}, 0, 0, 0, 16 },
        { ParamId_PARAM_DESCRIPTION, "desc", "Description", ParameterGroup::Information, ParamType_STRING, true, {}, 0, 0, 0, 32 },
        { ParamId_PARAM_SERIAL_NUMBER, "ser_num", "Serial Number", ParameterGroup::Information, ParamType_STRING, false, {}, 0, 0, 0, 16 },

        { ParamId_PARAM_DISK_UPDATE_RATE_MS, "dsk_updt", "Disk Sync Rate", ParameterGroup::SystemBehavior, ParamType_UINT32, false, "ms", 0, 0, 60000, 32 },
        { ParamId_PARAM_ACTIVITY_LED_SCALER, "actv_led_scale", "Heartbeat LED Rate", ParameterGroup::SystemBehavior, ParamType_FLOAT, false, "x", 3, 0, 100, 32 },
        { ParamId_PARAM_CAN_NODE_ID, "can_id", "CAN Node ID", ParameterGroup::SystemBehavior, ParamType_UINT8, false, {}, 0, 0, 255, 32 },

        { ParamId_PARAM_STATOR_PWM_FREQ, "stator_pwm_freq", "Stator PWM Frequency", ParameterGroup::MotorControl, ParamType_FLOAT, false, "Hz", 1, 0, 50000, 32 },
        { ParamId_PARAM_SPEED_CTRL_FREQ, "spd_ctrl_freq", "Speed Control Frequency", ParameterGroup::MotorControl, ParamType_FLOAT, false, "Hz", 1, 0, 20000, 32 },
        { ParamId_PARAM_TARGET_IDLE_RPM, "target_idle_rpm", "Target Idle RPM", ParameterGroup::MotorControl, ParamType_FLOAT, false, "rpm", 1, 0, 20000, 32 },
        { ParamId_PARAM_SPEED_CTRL_KP, "spd_ctrl_kp", "Speed Kp", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_SPEED_CTRL_KI, "spd_ctrl_ki", "Speed Ki", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_SPEED_CTRL_KD, "spd_ctrl_kd", "Speed Kd", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KP, "iq_ctrl_kp", "Q-Axis Kp", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KI, "iq_ctrl_ki", "Q-Axis Ki", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KD, "iq_ctrl_kd", "Q-Axis Kd", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_CTRL_D_AXIS_KP, "id_ctrl_kp", "D-Axis Kp", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_CTRL_D_AXIS_KI, "id_ctrl_ki", "D-Axis Ki", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_CTRL_D_AXIS_KD, "id_ctrl_kd", "D-Axis Kd", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_RAMP_CTRL_FIRST_ORDER_TERM, "ramp_1st_term", "Ramp First Order Term", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 4, -1000, 1000, 32 },
        { ParamId_PARAM_RAMP_CTRL_SECOND_ORDER_TERM, "ramp_2nd_term", "Ramp Second Order Term", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 4, -1000, 1000, 32 },
        { ParamId_PARAM_RAMP_CTRL_RAMP_TIME_SEC, "ramp_time_sec", "Ramp Time", ParameterGroup::MotorControl, ParamType_FLOAT, false, "s", 4, 0, 1000, 32 },
        { ParamId_PARAM_CURRENT_OBSERVER_KSLIDE, "smc_kslide", "Sliding Mode Gain", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_CURRENT_OBSERVER_MAX_ERROR, "smc_max_error", "Sliding Mode Max Error", ParameterGroup::MotorControl, ParamType_FLOAT, false, {}, 5, -1000, 1000, 32 },
        { ParamId_PARAM_PWM_DRIVE_MAX_DUTY, "pwm_max_duty", "PWM Max Duty", ParameterGroup::MotorControl, ParamType_FLOAT, false, "x", 4, 0, 1, 32 },

        { ParamId_PARAM_ROTOR_POLES, "rotor_poles", "Rotor Poles", ParameterGroup::MotorAttributes, ParamType_UINT8, false, {}, 0, 0, 255, 32 },
        { ParamId_PARAM_STATOR_SLOTS, "stator_slots", "Stator Slots", ParameterGroup::MotorAttributes, ParamType_UINT8, false, {}, 0, 0, 255, 32 },
        { ParamId_PARAM_STATOR_RESISTANCE, "stator_resistance", "Phase Resistance", ParameterGroup::MotorAttributes, ParamType_FLOAT, false, "Ohm", 5, 0, 1000, 32 },
        { ParamId_PARAM_STATOR_INDUCTANCE, "stator_inductance", "Phase Inductance", ParameterGroup::MotorAttributes, ParamType_FLOAT, false, "H", 5, 0, 10, 32 },
        { ParamId_PARAM_PEAK_CURRENT_THRESHOLD, "peak_current_threshold", "Peak Current Limit", ParameterGroup::MotorAttributes, ParamType_FLOAT, false, "A", 3, 0, 1000, 32 },
        { ParamId_PARAM_PEAK_VOLTAGE_THRESHOLD, "peak_voltage_threshold", "Peak Voltage Limit", ParameterGroup::MotorAttributes, ParamType_FLOAT, false, "V", 3, 0, 1000, 32 },
        { ParamId_PARAM_MIN_ARM_VOLTAGE, "min_arm_voltage", "Minimum Arm Voltage", ParameterGroup::MotorAttributes, ParamType_FLOAT, false, "V", 3, 0, 100, 32 },
        { ParamId_PARAM_MAX_ARM_VOLTAGE, "max_arm_voltage", "Maximum Arm Voltage", ParameterGroup::MotorAttributes, ParamType_FLOAT, false, "V", 3, 0, 100, 32 },
      };
      return list;
    }
  }    // namespace

  const QVector<ParameterDescriptor> &all()
  {
    return descriptors();
  }

  QVector<ParameterDescriptor> forGroup( ParameterGroup group )
  {
    QVector<ParameterDescriptor> result;
    for( const auto &item : descriptors() )
    {
      if( item.group == group )
      {
        result.append( item );
      }
    }
    return result;
  }

  QString groupName( ParameterGroup group )
  {
    switch( group )
    {
      case ParameterGroup::Information:
        return "Information";
      case ParameterGroup::SystemBehavior:
        return "System Behavior";
      case ParameterGroup::MotorControl:
        return "Motor Control";
      case ParameterGroup::MotorAttributes:
        return "Motor Attributes";
      default:
        return "Unknown";
    }
  }

  const ParameterDescriptor *find( int paramId )
  {
    for( const auto &item : descriptors() )
    {
      if( item.id == paramId )
      {
        return &item;
      }
    }
    return nullptr;
  }

  bool isValidParamId( int paramId )
  {
    return find( paramId ) != nullptr;
  }
}    // namespace orbit::model::registry
