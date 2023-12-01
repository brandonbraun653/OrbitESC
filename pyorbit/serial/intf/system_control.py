from __future__ import annotations

from enum import IntEnum
import pyorbit.nanopb.system_control_pb2 as proto
from pyorbit.nanopb import serial_interface_pb2 as proto
from pyorbit.serial.messages import BaseMessage, MessageSubId
from pyorbit.serial.serial_interface import MessageId


class SystemControlSubId(IntEnum):
    Reset = proto.SUB_MSG_SYS_CTRL_RESET
    Motor = proto.SUB_MSG_SYS_CTRL_MOTOR
    ManualInnerLoop = proto.SUB_MSG_SYS_CTRL_MANUAL_INNER_LOOP
    ManualInnerLoopRef = proto.SUB_MSG_SYS_CTRL_MANUAL_INNER_LOOP_REF


class StreamPhaseCurrentsMessage(BaseMessage):

    def __init__(self, enable: bool):
        super().__init__()
        self._pb_msg = proto.SystemControlMessage()
        self._pb_msg.header.msgId = MessageId.SystemCtrl.value

        if enable:
            self._pb_msg.header.subId = proto.SUB_MSG_ENABLE_STREAM_PHASE_CURRENTS
        else:
            self._pb_msg.header.subId = proto.SUB_MSG_DISABLE_STREAM_PHASE_CURRENTS


class MotorControlMessage(BaseMessage):

    class Command(IntEnum):
        """ Available commands for the motor control system message """
        EnableOutputStage = proto.ENABLE_OUTPUT_STAGE
        DisableOutputStage = proto.DISABLE_OUTPUT_STAGE
        EmergencyStop = proto.EMERGENCY_STOP

    def __init__(self, cmd: Command = None, data: bytes = None):
        super().__init__()
        self._pb_msg = proto.SystemControlMessage()
        self._pb_msg.header.msgId = MessageId.SystemCtrl.value
        self._pb_msg.header.subId = MessageSubId.SystemControl_Motor.value

        self._pb_msg.motorCmd = cmd
        self._pb_msg.data = data if data else b''

    @property
    def command(self) -> Command:
        return self._pb_msg.motorCmd

    @command.setter
    def command(self, cmd: Command):
        """
        Args:
            cmd: Command to assign, from protobuf MotorCtrlCmd enum
        """
        assert isinstance(cmd, MotorControlMessage.Command)
        self._pb_msg.motorCmd = cmd.value

    @property
    def payload(self) -> bytes:
        return self._pb_msg.data

    @payload.setter
    def payload(self, data: bytes):
        self._pb_msg.data = data


class SystemResetMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.SystemControlMessage()
        self._pb_msg.header.msgId = MessageId.SystemCtrl.value
        self._pb_msg.header.subId = MessageSubId.SystemControl_Reset.value


class ManualCurrentControlToggleMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.SystemControlMessage()
        self._pb_msg.header.msgId = MessageId.SystemCtrl.value
        self._pb_msg.header.subId = MessageSubId.SystemControl_ManualInnerLoop.value


class ManualCurrentControlSetPointMessage(BaseMessage):

    def __init__(self, id_ref: float, iq_ref: float, theta: float):
        super().__init__()
        self._pb_msg = proto.SystemControlMessage()
        self._pb_msg.header.msgId = MessageId.SystemCtrl.value
        self._pb_msg.header.subId = MessageSubId.SystemControl_ManualInnerLoopRef.value

        set_point = proto.SystemControlMessage.ManualICtrlSetPoint()
        set_point.rotor_theta_rad = theta
        set_point.id_ref = id_ref
        set_point.iq_ref = iq_ref
        self._pb_msg.data = bytes(set_point)
