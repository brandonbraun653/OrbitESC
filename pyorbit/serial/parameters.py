from __future__ import annotations

import struct
from enum import IntEnum
from typing import List, NewType, Union, Dict

from pyorbit.nanopb import serial_interface_pb2 as proto
from pyorbit.serial.messages import MessageSubId, AckNackMessage, BaseMessage, MessageId, PingMessage, \
    SystemTickMessage, ConsoleMessage, SystemDataMessage

# Encapsulate the Python parameter type in a new type, so we can use it in type hints
ParameterType = NewType('ParameterValue', Union[bool, int, float, str, bytes])


class MessageEncoding(IntEnum):
    """ Enum for the parameter encoding when talking to the remote device """

    Unknown = proto.UNKNOWN
    BOOL = proto.BOOL
    UINT8 = proto.UINT8
    UINT16 = proto.UINT16
    UINT32 = proto.UINT32
    FLOAT = proto.FLOAT
    DOUBLE = proto.DOUBLE
    BYTES = proto.BYTES
    STRING = proto.STRING

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


class ParamIOMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.ParamIOMessage()
        self._pb_msg.header.msgId = MessageId.ParamIO.value
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
    MessageId.AckNack: AckNackMessage,
    MessageId.PingCmd: PingMessage,
    MessageId.SystemTick: SystemTickMessage,
    MessageId.Terminal: ConsoleMessage,
    MessageId.ParamIO: ParamIOMessage,
    MessageId.SystemData: SystemDataMessage,
}


class SetActivityLedBlinkScalerMessage(BaseMessage):

    def __init__(self, scaler: float):
        super().__init__()
        self._pb_msg = proto.ParamIOMessage()
        self._pb_msg.header.msgId = MessageId.ParamIO.value
        self._pb_msg.header.subId = MessageSubId.ParamIO_Set.value
        self._pb_msg.header.uuid = self._id_gen.next_uuid
        self._pb_msg.id = ParameterId.ActivityLedScaler.value
        self._pb_msg.type = proto.FLOAT
        self._pb_msg.data = struct.pack('<f', scaler)
