from enum import IntEnum
import pyorbit.nanopb.motor_control_pb2 as proto


class MotorCtrlState(IntEnum):
    """ Enum of available motor controller states. """
    Invalid = proto.MOTOR_CTRL_STATE_INVALID
    Idle = proto.MOTOR_CTRL_STATE_IDLE
    Armed = proto.MOTOR_CTRL_STATE_ARMED
    Engaged = proto.MOTOR_CTRL_STATE_ENGAGED
    Fault = proto.MOTOR_CTRL_STATE_FAULT
