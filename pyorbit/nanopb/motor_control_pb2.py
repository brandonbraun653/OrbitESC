# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: motor_control.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='motor_control.proto',
  package='',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x13motor_control.proto\x1a\x0cnanopb.proto\"N\n\x13ManualICtrlSetPoint\x12\x17\n\x0frotor_theta_rad\x18\x01 \x02(\x02\x12\x0e\n\x06id_ref\x18\x02 \x02(\x02\x12\x0e\n\x06iq_ref\x18\x03 \x02(\x02*\x9f\x01\n\x0eMotorCtrlState\x12\x19\n\x15MOTOR_CTRL_STATE_IDLE\x10\x00\x12\x1a\n\x16MOTOR_CTRL_STATE_ARMED\x10\x01\x12\x1c\n\x18MOTOR_CTRL_STATE_ENGAGED\x10\x02\x12\x1a\n\x16MOTOR_CTRL_STATE_FAULT\x10\x03\x12\x1c\n\x18MOTOR_CTRL_STATE_INVALID\x10\x04*\x9e\x01\n\x0cMotorCtrlCmd\x12\x1a\n\x16MOTOR_CTRL_CMD_INVALID\x10\x00\x12&\n\"MOTOR_CTRL_CMD_ENABLE_OUTPUT_STAGE\x10\x01\x12\'\n#MOTOR_CTRL_CMD_DISABLE_OUTPUT_STAGE\x10\x02\x12!\n\x1dMOTOR_CTRL_CMD_EMERGENCY_STOP\x10\x03'
  ,
  dependencies=[nanopb__pb2.DESCRIPTOR,])

_MOTORCTRLSTATE = _descriptor.EnumDescriptor(
  name='MotorCtrlState',
  full_name='MotorCtrlState',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_IDLE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_ARMED', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_ENGAGED', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_FAULT', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_INVALID', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=118,
  serialized_end=277,
)
_sym_db.RegisterEnumDescriptor(_MOTORCTRLSTATE)

MotorCtrlState = enum_type_wrapper.EnumTypeWrapper(_MOTORCTRLSTATE)
_MOTORCTRLCMD = _descriptor.EnumDescriptor(
  name='MotorCtrlCmd',
  full_name='MotorCtrlCmd',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_CMD_INVALID', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_CMD_ENABLE_OUTPUT_STAGE', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_CMD_DISABLE_OUTPUT_STAGE', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_CMD_EMERGENCY_STOP', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=280,
  serialized_end=438,
)
_sym_db.RegisterEnumDescriptor(_MOTORCTRLCMD)

MotorCtrlCmd = enum_type_wrapper.EnumTypeWrapper(_MOTORCTRLCMD)
MOTOR_CTRL_STATE_IDLE = 0
MOTOR_CTRL_STATE_ARMED = 1
MOTOR_CTRL_STATE_ENGAGED = 2
MOTOR_CTRL_STATE_FAULT = 3
MOTOR_CTRL_STATE_INVALID = 4
MOTOR_CTRL_CMD_INVALID = 0
MOTOR_CTRL_CMD_ENABLE_OUTPUT_STAGE = 1
MOTOR_CTRL_CMD_DISABLE_OUTPUT_STAGE = 2
MOTOR_CTRL_CMD_EMERGENCY_STOP = 3



_MANUALICTRLSETPOINT = _descriptor.Descriptor(
  name='ManualICtrlSetPoint',
  full_name='ManualICtrlSetPoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='rotor_theta_rad', full_name='ManualICtrlSetPoint.rotor_theta_rad', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='id_ref', full_name='ManualICtrlSetPoint.id_ref', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='iq_ref', full_name='ManualICtrlSetPoint.iq_ref', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=37,
  serialized_end=115,
)

DESCRIPTOR.message_types_by_name['ManualICtrlSetPoint'] = _MANUALICTRLSETPOINT
DESCRIPTOR.enum_types_by_name['MotorCtrlState'] = _MOTORCTRLSTATE
DESCRIPTOR.enum_types_by_name['MotorCtrlCmd'] = _MOTORCTRLCMD
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ManualICtrlSetPoint = _reflection.GeneratedProtocolMessageType('ManualICtrlSetPoint', (_message.Message,), {
  'DESCRIPTOR' : _MANUALICTRLSETPOINT,
  '__module__' : 'motor_control_pb2'
  # @@protoc_insertion_point(class_scope:ManualICtrlSetPoint)
  })
_sym_db.RegisterMessage(ManualICtrlSetPoint)


# @@protoc_insertion_point(module_scope)
