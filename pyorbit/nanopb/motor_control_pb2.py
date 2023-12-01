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
  serialized_pb=b'\n\x13motor_control.proto\x1a\x0cnanopb.proto*\x9f\x01\n\x0eMotorCtrlState\x12\x1c\n\x18MOTOR_CTRL_STATE_INVALID\x10\x00\x12\x19\n\x15MOTOR_CTRL_STATE_IDLE\x10\x01\x12\x1a\n\x16MOTOR_CTRL_STATE_ARMED\x10\x02\x12\x1c\n\x18MOTOR_CTRL_STATE_ENGAGED\x10\x03\x12\x1a\n\x16MOTOR_CTRL_STATE_FAULT\x10\x04'
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
      name='MOTOR_CTRL_STATE_INVALID', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_IDLE', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_ARMED', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_ENGAGED', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOTOR_CTRL_STATE_FAULT', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=38,
  serialized_end=197,
)
_sym_db.RegisterEnumDescriptor(_MOTORCTRLSTATE)

MotorCtrlState = enum_type_wrapper.EnumTypeWrapper(_MOTORCTRLSTATE)
MOTOR_CTRL_STATE_INVALID = 0
MOTOR_CTRL_STATE_IDLE = 1
MOTOR_CTRL_STATE_ARMED = 2
MOTOR_CTRL_STATE_ENGAGED = 3
MOTOR_CTRL_STATE_FAULT = 4


DESCRIPTOR.enum_types_by_name['MotorCtrlState'] = _MOTORCTRLSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
