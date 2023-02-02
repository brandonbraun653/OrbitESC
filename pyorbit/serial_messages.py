# **********************************************************************************************************************
#   FileName:
#       serial_messages.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   01/21/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import pyorbit.nanopb.serial_interface_pb2 as proto
from google.protobuf.message import Message
from enum import IntEnum
from pyorbit.utils import Singleton
from threading import RLock


class MessageId(IntEnum):
    AckNack = proto.MSG_ACK_NACK
    PingCmd = proto.MSG_PING_CMD
    Terminal = proto.MSG_TERMINAL
    SystemTick = proto.MSG_SYS_TICK
    ParamIO = proto.MSG_PARAM_IO


class MessageSubId(IntEnum):
    ParamIO_Put = proto.SUB_MSG_PARAM_IO_PUT
    ParamIO_Get = proto.SUB_MSG_PARAM_IO_GET
    ParamIO_Sync = proto.SUB_MSG_PARAM_IO_SYNC
    ParamIO_Load = proto.SUB_MSG_PARAM_IO_LOAD


class ParameterId(IntEnum):
    BootCount = proto.PARAM_BOOT_COUNT


class UUIDGenerator(metaclass=Singleton):

    def __init__(self):
        self._uuid = 0
        self._lock = RLock()

    @property
    def next_uuid(self) -> int:
        with self._lock:
            self._uuid = (self._uuid + 1) % 256
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
        return self._pb_msg.header.uuid

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
        return self.pb_message.SerializeToString()


class AckNackMessage(BaseMessage):

    def __init__(self):
        super().__init__()

        self._pb_msg = proto.AckNackMessage()
        self._pb_msg.header.msgId = MessageId.AckNack.value


class PingMessage(BaseMessage):

    def __init__(self):
        super().__init__()

        self._pb_msg = proto.PingMessage()
        self._pb_msg.header.msgId = MessageId.PingCmd.value
        self._pb_msg.header.subId = 0
        self._pb_msg.header.uuid = self._id_gen.next_uuid


class SystemTick(BaseMessage):

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
        self._pb_msg.header.msgId = MessageId.SystemTick.value

    @property
    def frame_number(self) -> int:
        return self._pb_msg.this_frame

    @property
    def total_frames(self) -> int:
        return self._pb_msg.total_frames

    @property
    def data(self) -> bytes:
        return self._pb_msg.data


class ParamIOMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.ParamIOMessage()
        self._pb_msg.header.msgId = MessageId.ParamIO.value
        self._pb_msg.header.uuid = self._id_gen.next_uuid

    @property
    def sub_id(self) -> MessageSubId:
        return MessageSubId(self._pb_msg.header.sub_id)


MessageTypeMap = {
    MessageId.AckNack: AckNackMessage,
    MessageId.PingCmd: PingMessage,
    MessageId.SystemTick: SystemTick,
    MessageId.Terminal: ConsoleMessage,
    MessageId.ParamIO: ParamIOMessage
}
