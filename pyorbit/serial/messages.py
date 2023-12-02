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
from typing import Union
from enum import IntEnum
from google.protobuf.message import Message

from pyorbit.serial.serial_interface import MessageId
from pyorbit.utils import Singleton
from threading import RLock


class MessageSubId(IntEnum):
    Empty = proto.SUB_MSG_NONE

    # Parameter IO
    ParamIO_Set = proto.SUB_MSG_PARAM_IO_SET
    ParamIO_Get = proto.SUB_MSG_PARAM_IO_GET
    ParamIO_Sync = proto.SUB_MSG_PARAM_IO_SYNC
    ParamIO_Load = proto.SUB_MSG_PARAM_IO_LOAD


class StatusCode(IntEnum):
    NO_ERROR = proto.NO_ERROR
    UNKNOWN_ERROR = proto.UNKNOWN_ERROR
    INVALID_PARAM = proto.INVALID_PARAM
    INVALID_TYPE = proto.INVALID_TYPE
    INVALID_VALUE = proto.INVALID_VALUE
    REQUEST_FAILED = proto.REQUEST_FAILED


class Mode(IntEnum):
    Normal = proto.BOOT_MODE_NORMAL
    Test = proto.BOOT_MODE_TEST
    Config = proto.BOOT_MODE_CONFIG


class SystemDataId(IntEnum):
    SYS_DATA_INVALID = proto.SYS_DATA_INVALID
    ADC_PHASE_CURRENTS = proto.ADC_PHASE_CURRENTS
    ADC_PHASE_VOLTAGES = proto.ADC_PHASE_VOLTAGES
    ADC_SYSTEM_VOLTAGES = proto.ADC_SYSTEM_VOLTAGES
    STATE_ESTIMATES = proto.STATE_ESTIMATES


class UUIDGenerator(metaclass=Singleton):

    def __init__(self):
        self._uuid = 0
        self._lock = RLock()

    @property
    def next_uuid(self) -> int:
        with self._lock:
            self._uuid = (self._uuid + 1) % 65536
            return self._uuid


class BaseMessage:

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
    def sub_id(self) -> MessageSubId:
        return MessageSubId(self._pb_msg.header.subId)

    @sub_id.setter
    def sub_id(self, sid: MessageSubId) -> None:
        self._pb_msg.header.subId = sid.value

    @property
    def msg_id(self) -> MessageId:
        return self._pb_msg.header.msgId

    @msg_id.setter
    def msg_id(self, mid: MessageId):
        self._pb_msg.header.msgId = mid.value

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


class AckNackMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.AckNackMessage()
        self._pb_msg.header.msgId = MessageId.AckNack.value

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
        self._pb_msg.status_code = sc.value


class PingMessage(BaseMessage):

    def __init__(self):
        super().__init__()

        self._pb_msg = proto.PingMessage()
        self._pb_msg.header.msgId = MessageId.PingCmd.value
        self._pb_msg.header.subId = 0


class SystemTickMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.SystemTick()
        self._pb_msg.header.msgId = MessageId.SystemTick.value
        self._pb_msg.tick = 0

    @property
    def tick(self) -> int:
        return self._pb_msg.tick


class ConsoleMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.ConsoleMessage()
        self._pb_msg.header.msgId = MessageId.Terminal.value

    @property
    def frame_number(self) -> int:
        return self._pb_msg.this_frame

    @property
    def total_frames(self) -> int:
        return self._pb_msg.total_frames

    @property
    def data(self) -> bytes:
        return self._pb_msg.data


class SystemDataMessage(BaseMessage):

    class ADCPhaseCurrents(ctypes.Structure):
        _fields_ = [
            ('timestamp', ctypes.c_uint32),
            ('ia', ctypes.c_float),
            ('ib', ctypes.c_float),
            ('ic', ctypes.c_float),
        ]

    class ADCPhaseVoltages(ctypes.Structure):
        _fields_ = [
            ('timestamp', ctypes.c_uint32),
            ('va', ctypes.c_float),
            ('vb', ctypes.c_float),
            ('vc', ctypes.c_float),
        ]

    class ADCSystemVoltages(ctypes.Structure):
        _fields_ = [
            ('timestamp', ctypes.c_uint32),
            ('v_mcu', ctypes.c_float),
            ('v_dc_link', ctypes.c_float),
            ('v_temp', ctypes.c_float),
            ('v_isense', ctypes.c_float)
        ]

    class StateEstimates(ctypes.Structure):
        _fields_ = [
            ('timestamp', ctypes.c_uint32),
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
        self._pb_msg = proto.SystemDataMessage()
        self._pb_msg.header.msgId = MessageId.SystemData.value
        self._pb_msg.header.subId = 0

    @property
    def data_id(self) -> SystemDataId:
        return SystemDataId(self._pb_msg.id)

    @data_id.setter
    def data_id(self, data_id: SystemDataId):
        self._pb_msg.id = data_id.value

    @property
    def data(self) -> bytes:
        return self._pb_msg.data

    @data.setter
    def data(self, data: bytes):
        self._pb_msg.data = data

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
