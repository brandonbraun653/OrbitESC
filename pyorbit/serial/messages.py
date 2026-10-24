# **********************************************************************************************************************
#   FileName:
#       messages.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   01/21/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from __future__ import annotations

from threading import RLock
from typing import TypeVar, Optional, cast

from google.protobuf.message import Message
from loguru import logger

import pyorbit.nanopb.serial_interface_pb2 as proto
from pyorbit.nanopb.motor_control_pb2 import *
from pyorbit.nanopb.serial_interface_pb2 import *
from pyorbit.nanopb.system_control_pb2 import *
from pyorbit.nanopb.system_data_pb2 import *
from pyorbit.utils import Singleton


class UUIDGenerator(metaclass=Singleton):

    def __init__(self):
        self._uuid = 0
        self._lock = RLock()

    @property
    def next_uuid(self) -> int:
        with self._lock:
            self._uuid = (self._uuid + 1) % 65536
            return self._uuid


# Core type for messages that inherit from this class
BasePBMsgInheritor = TypeVar('BasePBMsgInheritor', bound='BasePBMsg')


class BasePBMsg:

    def __init__(self):
        self._id_gen = UUIDGenerator()
        self._pb_msg = None

    @property
    def name(self) -> str:
        return self.__class__.__name__

    @property
    def pb_message(self) -> Message:
        return self._pb_msg

    @property
    def uuid(self) -> int:
        self._assign_uuid_if_empty()
        return self._pb_msg.header.uuid

    @property
    def sub_id(self) -> int:
        return int(self._pb_msg.header.subId)

    @sub_id.setter
    def sub_id(self, sid: int) -> None:
        self._pb_msg.header.subId = sid

    @property
    def msg_id(self) -> MsgId:
        return self._pb_msg.header.msgId

    @msg_id.setter
    def msg_id(self, mid: MsgId):
        self._pb_msg.header.msgId = mid

    def deserialize(self, serialized: bytes) -> int:
        """
        Decodes a number of bytes into the message type for this class
        Args:
            serialized: Buffer of data containing the message to decode

        Returns:
            How many bytes were parsed
        """
        return self.pb_message.ParseFromString(serialized)

    def serialize(self) -> bytes:
        """
        Returns:
            Serialized message
        """
        self._assign_uuid_if_empty()
        return self.pb_message.SerializeToString()

    def _assign_uuid_if_empty(self):
        """
        ProtoBuf don't support default values for fields, but they do set them to 0 by default. We can
        use this to our advantage as a way to determine if the UUID has been set yet.

        Returns:
            None
        """
        if self._pb_msg and self._pb_msg.header.uuid == 0:
            self._pb_msg.header.uuid = self._id_gen.next_uuid


class AckNackPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.AckNackMessage()
        self._pb_msg.header.msgId = MsgId.MSG_ACK_NACK

    @property
    def ack(self) -> bool:
        return self._pb_msg.acknowledge

    @property
    def nack(self) -> bool:
        return not self._pb_msg.acknowledge

    @property
    def status_code(self) -> StatusCode:
        return self._pb_msg.status_code

    @status_code.setter
    def status_code(self, sc: StatusCode):
        self._pb_msg.status_code = sc


class PingPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()

        self._pb_msg = proto.PingMessage()
        self._pb_msg.header.msgId = MsgId.MSG_PING_CMD
        self._pb_msg.header.subId = 0


class SystemTickPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemTickMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_TICK
        self._pb_msg.tick = 0

    @property
    def tick(self) -> int:
        return self._pb_msg.tick


class SystemStatusPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemStatusMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_STATUS

    @property
    def tick(self) -> int:
        return self._pb_msg.systemTick

    @property
    def motor_ctrl_state(self) -> MotorCtrlState:
        return self._pb_msg.motorCtrlState


class SystemControlPbMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemControlMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_CTRL
        self._pb_msg.header.subId = SubId.SUB_MSG_NONE

    @property
    def message_type(self) -> SystemControlSubId:
        return cast(SystemControlSubId, self._pb_msg.header.subId)

    @message_type.setter
    def message_type(self, msg_type: SystemControlSubId):
        self._pb_msg.header.subId = msg_type


class ConsolePBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = ConsoleMessage()
        self._pb_msg.header.msgId = MsgId.MSG_TERMINAL

    @property
    def frame_number(self) -> int:
        return self._pb_msg.this_frame

    @property
    def total_frames(self) -> int:
        return self._pb_msg.total_frames

    @property
    def data(self) -> bytes:
        return self._pb_msg.data


class SystemDataPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemDataMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_DATA
        self._pb_msg.header.subId = 0

    @property
    def timestamp(self) -> int:
        """
        Returns:
            Device system time in microseconds when the data was captured
        """
        return self._pb_msg.timestamp

    @property
    def data_id(self) -> SystemDataId:
        return self._pb_msg.id

    @data_id.setter
    def data_id(self, data_id: SystemDataId):
        self._pb_msg.id = data_id

    @property
    def data(self) -> bytes:
        return self._pb_msg.payload

    @data.setter
    def data(self, data: bytes):
        self._pb_msg.payload = data

    def extract_payload(self) -> Optional[Message]:
        """
        Converts the data field to the appropriate message type based on the data_id field.
        Returns:
            The converted message type, or None if the data_id is not recognized.
        """
        _id_to_type = {
            SystemDataId.ADC_PHASE_CURRENTS: ADCPhaseCurrentsPayload,
            SystemDataId.ADC_PHASE_VOLTAGES: ADCPhaseVoltagesPayload,
            SystemDataId.ADC_SYSTEM_VOLTAGES: ADCSystemVoltagesPayload,
            SystemDataId.CURRENT_CONTROL_MONITOR: CurrentControlMonitorPayload,
            SystemDataId.SYSTEM_OBSERVER_MONITOR: SystemObserverMonitorPayload,
            SystemDataId.INNER_LOOP_VOLTAGES: InnerLoopVoltageMonitorPayload,
        }

        try:
            instance = _id_to_type[self.data_id]()
            instance.ParseFromString(self.data)
            return instance
        except KeyError:
            logger.error(f"Unknown data_id: {self.data_id}. Add to the mapping SystemDataPBMsg.extract_payload().")
            return None
