from __future__ import annotations
from enum import IntEnum
from typing import List
from pyorbit.nanopb import serial_interface_pb2 as proto


class ParameterType(IntEnum):
    Unknown = proto.UNKNOWN
    BOOL = proto.BOOL
    UINT8 = proto.UINT8
    UINT16 = proto.UINT16
    UINT32 = proto.UINT32
    FLOAT = proto.FLOAT
    DOUBLE = proto.DOUBLE
    BYTES = proto.BYTES
    STRING = proto.STRING


class ParameterId(IntEnum):

    @classmethod
    def values(cls) -> List[ParameterId]:
        all_values = list(cls.__members__.values())
        all_values.remove(ParameterId.Invalid)
        return all_values

    # Housekeeping parameters
    Invalid = proto.PARAM_INVALID

    # Read Only Parameters
    BootCount = proto.PARAM_BOOT_COUNT
    HwVersion = proto.PARAM_HW_VERSION
    SwVersion = proto.PARAM_SW_VERSION
    DeviceId = proto.PARAM_DEVICE_ID
    BoardName = proto.PARAM_BOARD_NAME
    Description = proto.PARAM_DESCRIPTION

    # Read/Write Parameters
    SerialNumber = proto.PARAM_SERIAL_NUMBER
    DiskUpdateRateMS = proto.PARAM_DISK_UPDATE_RATE_MS
    ActivityLedScaler = proto.PARAM_ACTIVITY_LED_SCALER
    BootMode = proto.PARAM_BOOT_MODE
    CanNodeId = proto.PARAM_CAN_NODE_ID

    # Motor Control Parameters
    StatorPWMFrequency = proto.PARAM_STATOR_PWM_FREQ
    SpeedControlFrequency = proto.PARAM_SPEED_CTRL_FREQ
    TargetIdleRPM = proto.PARAM_TARGET_IDLE_RPM
    SpeedControlKp = proto.PARAM_SPEED_CTRL_KP
    SpeedControlKi = proto.PARAM_SPEED_CTRL_KI
    SpeedControlKd = proto.PARAM_SPEED_CTRL_KD
    CurrentControlQKp = proto.PARAM_CURRENT_CTRL_Q_AXIS_KP
    CurrentControlQKi = proto.PARAM_CURRENT_CTRL_Q_AXIS_KI
    CurrentControlQKd = proto.PARAM_CURRENT_CTRL_Q_AXIS_KD
    CurrentControlDKp = proto.PARAM_CURRENT_CTRL_D_AXIS_KP
    CurrentControlDKi = proto.PARAM_CURRENT_CTRL_D_AXIS_KI
    CurrentControlDKd = proto.PARAM_CURRENT_CTRL_D_AXIS_KD
    RampControlFirstOrderTerm = proto.PARAM_RAMP_CTRL_FIRST_ORDER_TERM
    RampControlSecondOrderTerm = proto.PARAM_RAMP_CTRL_SECOND_ORDER_TERM
    RampControlRampTime = proto.PARAM_RAMP_CTRL_RAMP_TIME_SEC
    SlidingModeControlGain = proto.PARAM_CURRENT_OBSERVER_KSLIDE
    SlidingModeControlMaxError = proto.PARAM_CURRENT_OBSERVER_MAX_ERROR

    # Motor Description Parameters
    RotorPoles = proto.PARAM_ROTOR_POLES
    StatorSlots = proto.PARAM_STATOR_SLOTS
    StatorResistance = proto.PARAM_STATOR_RESISTANCE
    StatorInductance = proto.PARAM_STATOR_INDUCTANCE

    # Monitor Thresholds
    PeakCurrentThreshold = proto.PARAM_PEAK_CURRENT_THRESHOLD
    PeakVoltageThreshold = proto.PARAM_PEAK_VOLTAGE_THRESHOLD

    # System Behavior
    StreamPhaseCurrents = proto.PARAM_STREAM_PHASE_CURRENTS
    StreamPWMCommands = proto.PARAM_STREAM_PWM_COMMANDS
    StreamStateEstimates = proto.PARAM_STREAM_STATE_ESTIMATES


ParameterTypeMap = {
    ParameterId.SerialNumber: ParameterType.STRING,
    ParameterId.CanNodeId: ParameterType.UINT8,

    # Motor Control Parameters
    ParameterId.StatorPWMFrequency: ParameterType.FLOAT,
    ParameterId.SpeedControlFrequency: ParameterType.FLOAT,
    ParameterId.TargetIdleRPM: ParameterType.FLOAT,
    ParameterId.SpeedControlKp: ParameterType.FLOAT,
    ParameterId.SpeedControlKi: ParameterType.FLOAT,
    ParameterId.SpeedControlKd: ParameterType.FLOAT,
    ParameterId.CurrentControlQKp: ParameterType.FLOAT,
    ParameterId.CurrentControlQKi: ParameterType.FLOAT,
    ParameterId.CurrentControlQKd: ParameterType.FLOAT,
    ParameterId.CurrentControlDKp: ParameterType.FLOAT,
    ParameterId.CurrentControlDKi: ParameterType.FLOAT,
    ParameterId.CurrentControlDKd: ParameterType.FLOAT,
    ParameterId.RampControlFirstOrderTerm: ParameterType.FLOAT,
    ParameterId.RampControlSecondOrderTerm: ParameterType.FLOAT,
    ParameterId.RampControlRampTime: ParameterType.FLOAT,
    ParameterId.SlidingModeControlGain: ParameterType.FLOAT,
    ParameterId.SlidingModeControlMaxError: ParameterType.FLOAT,

    # Motor Description Parameters
    ParameterId.RotorPoles: ParameterType.UINT8,
    ParameterId.StatorSlots: ParameterType.UINT8,
    ParameterId.StatorResistance: ParameterType.FLOAT,
    ParameterId.StatorInductance: ParameterType.FLOAT,

    # Monitor Thresholds
    ParameterId.PeakCurrentThreshold: ParameterType.FLOAT,
    ParameterId.PeakVoltageThreshold: ParameterType.FLOAT,

    # System Behavior
    ParameterId.StreamPhaseCurrents: ParameterType.BOOL,
    ParameterId.StreamPWMCommands: ParameterType.BOOL,
    ParameterId.StreamStateEstimates: ParameterType.BOOL,
}
