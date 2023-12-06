from __future__ import annotations

import struct
from enum import IntEnum
from typing import List, NewType, Union, Dict
from pyorbit.nanopb.system_config_pb2 import *
from pyorbit.serial.messages import AckNackPBMsg, BasePBMsg, PingPBMsg, \
    SystemTickPBMsg, ConsolePBMsg, SystemDataPBMsg, SystemStatusPBMsg
from pyorbit.nanopb.serial_interface_pb2 import *

# Encapsulate the Python parameter type in a new type, so we can use it in type hints
ParameterType = NewType('ParameterValue', Union[bool, int, float, str, bytes])


class MessageEncoding(IntEnum):
    """ Enum for the parameter encoding when talking to the remote device """

    Unknown = ParamType.UNKNOWN
    BOOL = ParamType.BOOL
    UINT8 = ParamType.UINT8
    UINT16 = ParamType.UINT16
    UINT32 = ParamType.UINT32
    FLOAT = ParamType.FLOAT
    DOUBLE = ParamType.DOUBLE
    BYTES = ParamType.BYTES
    STRING = ParamType.STRING

    @classmethod
    def py_types(cls) -> Dict[MessageEncoding, ParameterType]:
        """
        Returns:
            A dictionary mapping parameter ids to their python type.
        """
        return {
            cls.BOOL: bool,
            cls.UINT8: int,
            cls.UINT16: int,
            cls.UINT32: int,
            cls.FLOAT: float,
            cls.BYTES: bytes,
            cls.STRING: str,
        }

    @classmethod
    def as_proto_type(cls, value: ParameterType) -> MessageEncoding:
        """
        Returns:
            The nanopb type of the parameter.
        """
        if isinstance(value, bool):
            return cls.BOOL
        elif isinstance(value, int):
            return cls.UINT32
        elif isinstance(value, float):
            return cls.FLOAT
        elif isinstance(value, bytes):
            return cls.BYTES
        elif isinstance(value, str):
            return cls.STRING
        else:
            raise ValueError(f"Invalid parameter type: {type(value)}")

    def as_py_type(self) -> ParameterType:
        """
        Returns:
            The python type of the parameter.
        """
        return MessageEncoding.py_types()[self]


class ParameterId(IntEnum):
    """ All available parameters """

    @classmethod
    def values(cls) -> List[ParameterId]:
        all_values = list(cls.__members__.values())
        all_values.remove(ParameterId.Invalid)
        return all_values

    # Housekeeping parameters
    Invalid = ParamId.PARAM_INVALID

    # Read Only Parameters
    BootCount = ParamId.PARAM_BOOT_COUNT
    HwVersion = ParamId.PARAM_HW_VERSION
    SwVersion = ParamId.PARAM_SW_VERSION
    DeviceId = ParamId.PARAM_DEVICE_ID
    BoardName = ParamId.PARAM_BOARD_NAME
    Description = ParamId.PARAM_DESCRIPTION

    # Read/Write Parameters
    SerialNumber = ParamId.PARAM_SERIAL_NUMBER
    DiskUpdateRateMS = ParamId.PARAM_DISK_UPDATE_RATE_MS
    ActivityLedScaler = ParamId.PARAM_ACTIVITY_LED_SCALER
    BootMode = ParamId.PARAM_BOOT_MODE
    CanNodeId = ParamId.PARAM_CAN_NODE_ID

    # Motor Control Parameters
    StatorPWMFrequency = ParamId.PARAM_STATOR_PWM_FREQ
    SpeedControlFrequency = ParamId.PARAM_SPEED_CTRL_FREQ
    TargetIdleRPM = ParamId.PARAM_TARGET_IDLE_RPM
    SpeedControlKp = ParamId.PARAM_SPEED_CTRL_KP
    SpeedControlKi = ParamId.PARAM_SPEED_CTRL_KI
    SpeedControlKd = ParamId.PARAM_SPEED_CTRL_KD
    CurrentControlQKp = ParamId.PARAM_CURRENT_CTRL_Q_AXIS_KP
    CurrentControlQKi = ParamId.PARAM_CURRENT_CTRL_Q_AXIS_KI
    CurrentControlQKd = ParamId.PARAM_CURRENT_CTRL_Q_AXIS_KD
    CurrentControlDKp = ParamId.PARAM_CURRENT_CTRL_D_AXIS_KP
    CurrentControlDKi = ParamId.PARAM_CURRENT_CTRL_D_AXIS_KI
    CurrentControlDKd = ParamId.PARAM_CURRENT_CTRL_D_AXIS_KD
    RampControlFirstOrderTerm = ParamId.PARAM_RAMP_CTRL_FIRST_ORDER_TERM
    RampControlSecondOrderTerm = ParamId.PARAM_RAMP_CTRL_SECOND_ORDER_TERM
    RampControlRampTime = ParamId.PARAM_RAMP_CTRL_RAMP_TIME_SEC
    SlidingModeControlGain = ParamId.PARAM_CURRENT_OBSERVER_KSLIDE
    SlidingModeControlMaxError = ParamId.PARAM_CURRENT_OBSERVER_MAX_ERROR

    # Motor Description Parameters
    RotorPoles = ParamId.PARAM_ROTOR_POLES
    StatorSlots = ParamId.PARAM_STATOR_SLOTS
    StatorResistance = ParamId.PARAM_STATOR_RESISTANCE
    StatorInductance = ParamId.PARAM_STATOR_INDUCTANCE

    # Monitor Thresholds
    PeakCurrentThreshold = ParamId.PARAM_PEAK_CURRENT_THRESHOLD
    PeakVoltageThreshold = ParamId.PARAM_PEAK_VOLTAGE_THRESHOLD


class ParamIOPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = ParamIOMessage()
        self._pb_msg.header.msgId = MsgId.MSG_PARAM_IO
        self._pb_msg.header.uuid = self._id_gen.next_uuid

    @property
    def param_id(self) -> ParameterId:
        return ParameterId(self._pb_msg.id)

    @param_id.setter
    def param_id(self, pid: ParameterId):
        self._pb_msg.id = pid.value

    @property
    def param_type(self) -> MessageEncoding:
        return MessageEncoding(self._pb_msg.type)

    @param_type.setter
    def param_type(self, pt: MessageEncoding):
        self._pb_msg.type = pt.value

    @property
    def data(self) -> bytes:
        return self._pb_msg.data

    @data.setter
    def data(self, d: bytes):
        self._pb_msg.data = d


MessageTypeMap = {
    MsgId.MSG_ACK_NACK: AckNackPBMsg,
    MsgId.MSG_PING_CMD: PingPBMsg,
    MsgId.MSG_SYS_TICK: SystemTickPBMsg,
    MsgId.MSG_TERMINAL: ConsolePBMsg,
    MsgId.MSG_PARAM_IO: ParamIOPBMsg,
    MsgId.MSG_SYS_DATA: SystemDataPBMsg,
    MsgId.MSG_SYS_STATUS: SystemStatusPBMsg,
}


class SetActivityLedBlinkScalerPBMsg(BasePBMsg):

    def __init__(self, scaler: float):
        super().__init__()
        self._pb_msg = ParamIOMessage()
        self._pb_msg.header.msgId = MsgId.MSG_PARAM_IO
        self._pb_msg.header.subId = ParamIOSubId.SET
        self._pb_msg.header.uuid = self._id_gen.next_uuid
        self._pb_msg.id = ParameterId.ActivityLedScaler.value
        self._pb_msg.type = ParamType.FLOAT
        self._pb_msg.data = struct.pack('<f', scaler)
