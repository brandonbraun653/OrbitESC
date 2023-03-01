# **********************************************************************************************************************
#   FileName:
#       serial_messages.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   01/21/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import struct
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
    SystemCtrl = proto.MSG_SYS_CTRL
    SwitchMode = proto.MSG_SWITCH_MODE


class MessageSubId(IntEnum):
    # Parameter IO
    ParamIO_Set = proto.SUB_MSG_PARAM_IO_SET
    ParamIO_Get = proto.SUB_MSG_PARAM_IO_GET
    ParamIO_Sync = proto.SUB_MSG_PARAM_IO_SYNC
    ParamIO_Load = proto.SUB_MSG_PARAM_IO_LOAD

    # System Control
    SystemControl_Reset = proto.SUB_MSG_SYS_CTRL_RESET


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
        return self.pb_message.SerializeToString()


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
    def param_type(self) -> ParameterType:
        return ParameterType(self._pb_msg.type)

    @param_type.setter
    def param_type(self, pt: ParameterType):
        self._pb_msg.type = pt.value

    @property
    def data(self) -> bytes:
        return self._pb_msg.data

    @data.setter
    def data(self, d: bytes):
        self._pb_msg.data = d


class SystemResetMessage(BaseMessage):

    def __init__(self):
        super().__init__()
        self._pb_msg = proto.SystemControlMessage()
        self._pb_msg.header.msgId = MessageId.SystemCtrl.value
        self._pb_msg.header.subId = MessageSubId.SystemControl_Reset.value
        self._pb_msg.header.uuid = self._id_gen.next_uuid


class SetActivityLedBlinkScalerMessage(BaseMessage):

    def __init__(self, scaler: float):
        super().__init__()
        self._pb_msg = proto.ParamIOMessage()
        self._pb_msg.header.msgId = MessageId.ParamIO.value
        self._pb_msg.header.subId = MessageSubId.ParamIO_Put.value
        self._pb_msg.header.uuid = self._id_gen.next_uuid
        self._pb_msg.id = ParameterId.ActivityLedScaler.value
        self._pb_msg.type = ParameterType.FLOAT.value
        self._pb_msg.data = struct.pack('<f', scaler)


class SwitchModeMessage(BaseMessage):

    def __init__(self, mode: Mode):
        super().__init__()
        self._pb_msg = proto.SwitchModeMessage()
        self._pb_msg.header.msgId = MessageId.SwitchMode.value
        self._pb_msg.header.subId = 0
        self._pb_msg.header.uuid = self._id_gen.next_uuid
        self._pb_msg.mode = mode.value


# Maps message IDs to message class types
MessageTypeMap = {
    MessageId.AckNack: AckNackMessage,
    MessageId.PingCmd: PingMessage,
    MessageId.SystemTick: SystemTick,
    MessageId.Terminal: ConsoleMessage,
    MessageId.ParamIO: ParamIOMessage
}

# Maps parameter IDs to their associated types
ParameterTypeMap = {
    ParameterId.SerialNumber: ParameterType.STRING,
}
