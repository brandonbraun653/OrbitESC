"""
@generated by mypy-protobuf.  Do not edit manually!
isort:skip_file
"""
import builtins
import google.protobuf.descriptor
import google.protobuf.internal.enum_type_wrapper
import google.protobuf.message
import typing
import typing_extensions

DESCRIPTOR: google.protobuf.descriptor.FileDescriptor = ...

class _MsgId:
    ValueType = typing.NewType('ValueType', builtins.int)
    V: typing_extensions.TypeAlias = ValueType
class _MsgIdEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_MsgId.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor = ...
    MSG_ACK_NACK: MsgId.ValueType = ...  # 0
    """Generic ack/nack type message"""

    MSG_PING_CMD: MsgId.ValueType = ...  # 1
    """Simple PING to see if the node is alive"""

    MSG_TERMINAL: MsgId.ValueType = ...  # 2
    """Terminal command for printing text/debug data"""

    MSG_SYS_TICK: MsgId.ValueType = ...  # 3
    """System time tick"""

    MSG_SYS_INFO: MsgId.ValueType = ...  # 4
    """System information"""

    MSG_PARAM_IO: MsgId.ValueType = ...  # 5
    """Do operations on configurable parameters"""

    MSG_SYS_CTRL: MsgId.ValueType = ...  # 6
    """Perform system control operations"""

    MSG_SYS_DATA: MsgId.ValueType = ...  # 7
    """System data stream"""

    MSG_SYS_STATUS: MsgId.ValueType = ...  # 8
    """System status annunciation, essentially a snapshot of observable system state"""

class MsgId(_MsgId, metaclass=_MsgIdEnumTypeWrapper):
    """Message IDs for all the core message types sent between the host and the device."""
    pass

MSG_ACK_NACK: MsgId.ValueType = ...  # 0
"""Generic ack/nack type message"""

MSG_PING_CMD: MsgId.ValueType = ...  # 1
"""Simple PING to see if the node is alive"""

MSG_TERMINAL: MsgId.ValueType = ...  # 2
"""Terminal command for printing text/debug data"""

MSG_SYS_TICK: MsgId.ValueType = ...  # 3
"""System time tick"""

MSG_SYS_INFO: MsgId.ValueType = ...  # 4
"""System information"""

MSG_PARAM_IO: MsgId.ValueType = ...  # 5
"""Do operations on configurable parameters"""

MSG_SYS_CTRL: MsgId.ValueType = ...  # 6
"""Perform system control operations"""

MSG_SYS_DATA: MsgId.ValueType = ...  # 7
"""System data stream"""

MSG_SYS_STATUS: MsgId.ValueType = ...  # 8
"""System status annunciation, essentially a snapshot of observable system state"""

global___MsgId = MsgId


class _SubId:
    ValueType = typing.NewType('ValueType', builtins.int)
    V: typing_extensions.TypeAlias = ValueType
class _SubIdEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_SubId.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor = ...
    SUB_MSG_NONE: SubId.ValueType = ...  # 0
    """Invalid/empty sub-message ID"""

class SubId(_SubId, metaclass=_SubIdEnumTypeWrapper):
    """Specialization sub-IDs to further specify the root message ID."""
    pass

SUB_MSG_NONE: SubId.ValueType = ...  # 0
"""Invalid/empty sub-message ID"""

global___SubId = SubId


class _StatusCode:
    ValueType = typing.NewType('ValueType', builtins.int)
    V: typing_extensions.TypeAlias = ValueType
class _StatusCodeEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_StatusCode.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor = ...
    NO_ERROR: StatusCode.ValueType = ...  # 0
    UNKNOWN_ERROR: StatusCode.ValueType = ...  # 1
    INVALID_PARAM: StatusCode.ValueType = ...  # 2
    INVALID_TYPE: StatusCode.ValueType = ...  # 3
    INVALID_VALUE: StatusCode.ValueType = ...  # 4
    REQUEST_FAILED: StatusCode.ValueType = ...  # 5
class StatusCode(_StatusCode, metaclass=_StatusCodeEnumTypeWrapper):
    """Status codes for ACK/NACK messages"""
    pass

NO_ERROR: StatusCode.ValueType = ...  # 0
UNKNOWN_ERROR: StatusCode.ValueType = ...  # 1
INVALID_PARAM: StatusCode.ValueType = ...  # 2
INVALID_TYPE: StatusCode.ValueType = ...  # 3
INVALID_VALUE: StatusCode.ValueType = ...  # 4
REQUEST_FAILED: StatusCode.ValueType = ...  # 5
global___StatusCode = StatusCode


class Header(google.protobuf.message.Message):
    """Core message header common to all types. Each functional message type **must**
    have this first in their list of declarations.
    """
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    MSGID_FIELD_NUMBER: builtins.int
    SUBID_FIELD_NUMBER: builtins.int
    UUID_FIELD_NUMBER: builtins.int
    msgId: builtins.int = ...
    """Root message identifier"""

    subId: builtins.int = ...
    """Possible sub-identifier to specify root ID details"""

    uuid: builtins.int = ...
    """Unique ID for the message"""

    def __init__(self,
        *,
        msgId : typing.Optional[builtins.int] = ...,
        subId : typing.Optional[builtins.int] = ...,
        uuid : typing.Optional[builtins.int] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["msgId",b"msgId","subId",b"subId","uuid",b"uuid"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["msgId",b"msgId","subId",b"subId","uuid",b"uuid"]) -> None: ...
global___Header = Header

class BaseMessage(google.protobuf.message.Message):
    """Root type that parsers can use to peek at messages and figure out what type the full message is."""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> global___Header: ...
    def __init__(self,
        *,
        header : typing.Optional[global___Header] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["header",b"header"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["header",b"header"]) -> None: ...
global___BaseMessage = BaseMessage

class AckNackMessage(google.protobuf.message.Message):
    """Generic ACK or NACK to a previous message, with optional data payload"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    ACKNOWLEDGE_FIELD_NUMBER: builtins.int
    STATUS_CODE_FIELD_NUMBER: builtins.int
    DATA_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> global___Header: ...
    acknowledge: builtins.bool = ...
    """True if this is an ACK, false if it's a NACK"""

    status_code: global___StatusCode.ValueType = ...
    """Status code for the ACK/NACK"""

    data: builtins.bytes = ...
    """Optional data payload"""

    def __init__(self,
        *,
        header : typing.Optional[global___Header] = ...,
        acknowledge : typing.Optional[builtins.bool] = ...,
        status_code : typing.Optional[global___StatusCode.ValueType] = ...,
        data : typing.Optional[builtins.bytes] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["acknowledge",b"acknowledge","data",b"data","header",b"header","status_code",b"status_code"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["acknowledge",b"acknowledge","data",b"data","header",b"header","status_code",b"status_code"]) -> None: ...
global___AckNackMessage = AckNackMessage

class PingMessage(google.protobuf.message.Message):
    """Simple PING message to see if the node is alive, client or server can send this."""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> global___Header: ...
    def __init__(self,
        *,
        header : typing.Optional[global___Header] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["header",b"header"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["header",b"header"]) -> None: ...
global___PingMessage = PingMessage
