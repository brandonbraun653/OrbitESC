# **********************************************************************************************************************
#   FileName:
#       messages.py
#
#   Description:
#       Message types understood by an OrbitESC node over CAN bus
#
#   03/02/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from __future__ import annotations

import can
from ctypes import *
from enum import IntEnum
from typing import Any, Union
from loguru import logger


class NodeID(IntEnum):
    NODE_0 = 0,
    NODE_1 = 1,
    NODE_2 = 2,
    NODE_3 = 3,
    NODE_4 = 4,
    NODE_5 = 5,
    NODE_PC = 6


class BaseMessage(Structure):
    _pack_ = 1
    _period_ = 0
    _id_ = 0

    @classmethod
    def period(cls) -> int:
        """
        Returns:
            Expected message period interval in milliseconds
        """
        return cls._period_

    @classmethod
    def id(cls) -> int:
        """
        Returns:
            CAN bus ID of the message
        """
        return cls._id_

    def unpack(self, buffer: Union[bytes, can.Message]) -> BaseMessage:
        """
        Takes a buffer of data and unpacks it into the structure fields
        Args:
            buffer: Buffer to be unpacked

        Returns:
            None
        """
        if isinstance(buffer, can.Message):
            buffer = bytes(buffer.data)

        if len(buffer) < sizeof(self):
            raise ValueError(f"Buffer is too small to populate {self}")
        elif not isinstance(buffer, bytes):
            raise ValueError(f"Unpack expected a bytes input, not {type(buffer)}")

        memmove(addressof(self), buffer, sizeof(self))
        return self

    def pack(self) -> bytes:
        """
        Translates the message field into bytes for sending
        Returns:
            bytes
        """
        return bytes(self)

    def get_keyed_data(self, key: str) -> Any:
        """
        Simple lookup function for message types to retrieve arbitrary information
        Args:
            key: Which instance attribute to get

        Returns:
            Data associated with the key, else None
        """
        if hasattr(self, key):
            return getattr(self, key)
        else:
            logger.error(f"Object {repr(self)} has no attribute {key}")


class SystemID(BaseMessage):
    _fields_ = [("node_id", c_uint8, 3),
                ("_pad", c_uint8, 5)]


class Ping(BaseMessage):
    _fields_ = [("dst", SystemID),
                ("src", SystemID)]
    _id_ = 0x10


class SystemTick(BaseMessage):
    _fields_ = [("src", SystemID),
                ("tick", c_uint32)]
    _period_ = 1.0
    _id_ = 0x50


class PowerSupplyVoltage(BaseMessage):
    _fields_ = [("hdr", SystemID),
                ("timestamp", c_uint32),
                ("vdd", c_uint16)]
    _period_ = 0.1
    _id_ = 0x20

    def get_keyed_data(self, key: str) -> Any:
        if key == "vdd":
            return self.vdd / 1e3
        elif key == "timestamp":
            return self.timestamp / 1e6


class PhaseACurrent(BaseMessage):
    _fields_ = [("hdr", SystemID),
                ("timestamp", c_uint32),
                ("current", c_uint16)]
    _period_ = 0.1
    _id_ = 0x21


class PhaseBCurrent(BaseMessage):
    _fields_ = [("hdr", SystemID),
                ("timestamp", c_uint32),
                ("current", c_uint16)]
    _period_ = 0.1
    _id_ = 0x22


class MotorSpeed(BaseMessage):
    _fields_ = [("hdr", SystemID),
                ("tick", c_uint32),
                ("speed", c_uint16)]

    _period_ = 0.1
    _id_ = 0x23
