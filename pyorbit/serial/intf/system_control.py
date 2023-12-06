from __future__ import annotations

from pyorbit.serial.messages import BasePBMsg
from pyorbit.nanopb.serial_interface_pb2 import *
from pyorbit.nanopb.system_control_pb2 import *


class StreamPhaseCurrentsPBMsg(BasePBMsg):

    def __init__(self, enable: bool):
        super().__init__()
        self._pb_msg = SystemControlMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_CTRL

        if enable:
            self._pb_msg.header.subId = SystemControlSubId.ENABLE_STREAM_PHASE_CURRENTS
        else:
            self._pb_msg.header.subId = SystemControlSubId.DISABLE_STREAM_PHASE_CURRENTS


class SystemResetPBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemControlMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_CTRL
        self._pb_msg.header.subId = SystemControlSubId.RESET


class ManualCurrentControlTogglePBMsg(BasePBMsg):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemControlMessage()
        self._pb_msg.header.msgId = MsgId.MSG_SYS_CTRL
        self._pb_msg.header.subId = SystemControlSubId.MANUAL_INNER_LOOP
