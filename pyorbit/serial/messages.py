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

import ctypes
import pyorbit.nanopb.serial_interface_pb2 as proto
from pyorbit.nanopb.system_data_pb2 import *
from typing import Union, TypeVar
from google.protobuf.message import Message
from pyorbit.nanopb.serial_interface_pb2 import *
from pyorbit.utils import Singleton
from threading import RLock


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

    class ADCPhaseCurrents(ctypes.Structure):
        _fields_ = [
            ('ia', ctypes.c_float),
            ('ib', ctypes.c_float),
            ('ic', ctypes.c_float),
        ]

    class ADCPhaseVoltages(ctypes.Structure):
        _fields_ = [
            ('va', ctypes.c_float),
            ('vb', ctypes.c_float),
            ('vc', ctypes.c_float),
        ]

    class ADCSystemVoltages(ctypes.Structure):
        _fields_ = [
            ('v_mcu', ctypes.c_float),
            ('v_dc_link', ctypes.c_float),
            ('v_temp', ctypes.c_float),
            ('v_isense', ctypes.c_float)
        ]

    class StateEstimates(ctypes.Structure):
        _fields_ = [
            ('position', ctypes.c_float),
            ('speed', ctypes.c_float),
        ]

    _id_to_type = {
        SystemDataId.ADC_PHASE_CURRENTS: ADCPhaseCurrents,
        SystemDataId.ADC_PHASE_VOLTAGES: ADCPhaseVoltages,
        SystemDataId.ADC_SYSTEM_VOLTAGES: ADCSystemVoltages,
        SystemDataId.STATE_ESTIMATES: StateEstimates,
    }

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

    def convert_to_message_type(self) -> Union[None, ctypes.Structure]:
        """
        Converts the data field to the appropriate message type based on the data_id field.
        Returns:
            The converted message type, or None if the data_id is not recognized.
        """
        msg_type = self._id_to_type.get(self.data_id, None)
        if msg_type is None:
            return None
        return msg_type.from_buffer_copy(self.data)
