# **********************************************************************************************************************
#   FileName:
#       serial_pipe_framing.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   01/21/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import pyorbit.nanopb.serial_interface_pb2 as proto
from google.protobuf.message import Message
from enum import IntEnum


class MessageId(IntEnum):
    AckNack = 0
    PingCmd = 1
    Terminal = 2


class BaseMessage:

    def __init__(self):
        self._pb_msg = None

    @property
    def pb_message(self) -> Message:
        return self._pb_msg


class PingMessage(BaseMessage):

    def __init__(self):
        super().__init__()

        self._pb_msg = proto.PingMessage()
        self._pb_msg.header.msgId = MessageId.PingCmd.value
        self._pb_msg.header.subId = 0
        self._pb_msg.header.size = 0