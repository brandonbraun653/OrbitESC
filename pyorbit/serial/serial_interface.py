from __future__ import annotations

from enum import IntEnum

from pyorbit.nanopb import serial_interface_pb2 as proto


class MessageId(IntEnum):
    AckNack = proto.MSG_ACK_NACK
    PingCmd = proto.MSG_PING_CMD
    Terminal = proto.MSG_TERMINAL
    SystemTick = proto.MSG_SYS_TICK
    ParamIO = proto.MSG_PARAM_IO
    SystemCtrl = proto.MSG_SYS_CTRL
    SystemData = proto.MSG_SYS_DATA
