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
    NODE_PC = 4,
    NODE_ALL = 5


class Header(Structure):
    _fields_ = [("node_id", c_uint8, 3),
                ("size", c_uint8, 3),
                ("_pad", c_uint8, 2)]


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

    @classmethod
    def rate(cls) -> float:
        """
        Returns:
            Rate of the message in seconds
        """
        return cls._period_

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

    def as_bus_msg(self) -> can.Message:
        """
        Translates the current object into a formatted CAN bus message
        Returns:
            Message object ready for transmission
        """
        return can.Message(arbitration_id=self.id(), data=self.pack())

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


# -----------------------------------------------------------------------------
# Asynchronous Messages
# -----------------------------------------------------------------------------
class Ping(BaseMessage):
    _fields_ = [("dst", Header),
                ("src", Header)]
    _id_ = 0x10


class SetSystemMode(BaseMessage):
    _fields_ = [("dst", Header),
                ("mode", c_uint8)]
    _id_ = 0x11


class SetMotorSpeed(BaseMessage):
    _fields_ = [("dst", Header),
                ("speed", c_uint16)]
    _id_ = 0x12


class SetConfigData(BaseMessage):
    _fields_ = [("dst", Header),
                ("id", c_uint8),
                ("data", c_uint8 * 6)]
    _id_ = 0x13


class GetConfigData(BaseMessage):
    _fields_ = [("dst", Header),
                ("id", c_uint8)]
    _id_ = 0x14


class RspConfigData(BaseMessage):
    _fields_ = [("src", Header),
                ("id", c_uint8),
                ("data", c_uint8 * 6)]
    _id_ = 0x15


class EmergencyHalt(BaseMessage):
    _fields_ = [("dst", Header)]
    _id_ = 0x16


class SystemReset(BaseMessage):
    _fields_ = [("dst", Header)]
    _id_ = 0x17


# -----------------------------------------------------------------------------
# Periodic Messages
# -----------------------------------------------------------------------------
class SystemTick(BaseMessage):
    _fields_ = [("src", Header),
                ("tick", c_uint32)]
    _period_ = 1.0
    _id_ = 0x50


class SystemMode(BaseMessage):
    _fields_ = [("hdr", Header),
                ("mode", c_uint8)]
    _period_ = 0.250
    _id_ = 0x51


class PowerSupplyVoltage(BaseMessage):
    _fields_ = [("hdr", Header),
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
    _fields_ = [("hdr", Header),
                ("timestamp", c_uint32),
                ("current", c_uint16)]
    _period_ = 0.1
    _id_ = 0x21


class PhaseBCurrent(BaseMessage):
    _fields_ = [("hdr", Header),
                ("timestamp", c_uint32),
                ("current", c_uint16)]
    _period_ = 0.1
    _id_ = 0x22


class MotorSpeed(BaseMessage):
    _fields_ = [("hdr", Header),
                ("tick", c_uint32),
                ("speed", c_uint16)]

    _period_ = 0.1
    _id_ = 0x23


class SpeedReference(BaseMessage):
    _fields_ = [("hdr", Header),
                ("tick", c_uint32),
                ("speed", c_uint16)]

    _period_ = 0.1
    _id_ = 0x24
