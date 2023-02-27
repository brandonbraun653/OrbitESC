# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: serial_interface.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='serial_interface.proto',
  package='',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x16serial_interface.proto\x1a\x0cnanopb.proto\"I\n\x06Header\x12\x14\n\x05msgId\x18\x01 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05subId\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x13\n\x04uuid\x18\x03 \x02(\rB\x05\x92?\x02\x38\x10\"&\n\x0b\x42\x61seMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\"g\n\x0e\x41\x63kNackMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x13\n\x0b\x61\x63knowledge\x18\x02 \x02(\x08\x12\'\n\x0bstatus_code\x18\x03 \x02(\x0e\x32\x0b.StatusCodeB\x05\x92?\x02\x38\x08\"&\n\x0bPingMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\"3\n\nSystemTick\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x0c\n\x04tick\x18\x02 \x02(\r\"w\n\x0e\x43onsoleMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x19\n\nthis_frame\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x1b\n\x0ctotal_frames\x18\x03 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x04\x64\x61ta\x18\x04 \x02(\x0c\x42\x06\x92?\x03\x08\x80\x01\"\xa2\x01\n\x11SystemInfoMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x12\n\nsystemTick\x18\x02 \x02(\r\x12\x1d\n\tswVersion\x18\x03 \x02(\tB\n\x92?\x02\x08\x10\x92?\x02x\x01\x12\x1f\n\x0b\x64\x65scription\x18\x04 \x02(\tB\n\x92?\x02\x08\x10\x92?\x02x\x01\x12 \n\x0cserialNumber\x18\x05 \x02(\tB\n\x92?\x02\x08\x10\x92?\x02x\x01\"|\n\x0eParamIOMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x1b\n\x02id\x18\x02 \x01(\x0e\x32\x08.ParamIdB\x05\x92?\x02\x38\x10\x12\x1f\n\x04type\x18\x03 \x01(\x0e\x32\n.ParamTypeB\x05\x92?\x02\x38\x08\x12\x13\n\x04\x64\x61ta\x18\x04 \x01(\x0c\x42\x05\x92?\x02\x08@\"/\n\x14SystemControlMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header*\x85\x01\n\x05MsgId\x12\x10\n\x0cMSG_ACK_NACK\x10\x00\x12\x10\n\x0cMSG_PING_CMD\x10\x01\x12\x10\n\x0cMSG_TERMINAL\x10\x02\x12\x10\n\x0cMSG_SYS_TICK\x10\x03\x12\x10\n\x0cMSG_SYS_INFO\x10\x04\x12\x10\n\x0cMSG_PARAM_IO\x10\x05\x12\x10\n\x0cMSG_SYS_CTRL\x10\x06*\x91\x01\n\x05SubId\x12\x18\n\x14SUB_MSG_PARAM_IO_GET\x10\x00\x12\x18\n\x14SUB_MSG_PARAM_IO_PUT\x10\x01\x12\x19\n\x15SUB_MSG_PARAM_IO_SYNC\x10\x02\x12\x19\n\x15SUB_MSG_PARAM_IO_LOAD\x10\x03\x12\x1a\n\x16SUB_MSG_SYS_CTRL_RESET\x10\x00\x1a\x02\x10\x01*\x95\x02\n\x07ParamId\x12\x1a\n\rPARAM_INVALID\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\x12\x14\n\x10PARAM_BOOT_COUNT\x10\x00\x12\x14\n\x10PARAM_HW_VERSION\x10\x01\x12\x14\n\x10PARAM_SW_VERSION\x10\x02\x12\x13\n\x0fPARAM_DEVICE_ID\x10\x03\x12\x14\n\x10PARAM_BOARD_NAME\x10\x04\x12\x15\n\x11PARAM_DESCRIPTION\x10\x05\x12\x17\n\x13PARAM_SERIAL_NUMBER\x10\n\x12\x1d\n\x19PARAM_DISK_UPDATE_RATE_MS\x10\x0b\x12\x1d\n\x19PARAM_ACTIVITY_LED_SCALER\x10\x0c\x12\x13\n\x0fPARAM_BOOT_MODE\x10\r*s\n\tParamType\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x08\n\x04\x42OOL\x10\x01\x12\t\n\x05UINT8\x10\x02\x12\n\n\x06UINT16\x10\x03\x12\n\n\x06UINT32\x10\x04\x12\t\n\x05\x46LOAT\x10\x05\x12\n\n\x06\x44OUBLE\x10\x06\x12\t\n\x05\x42YTES\x10\x07\x12\n\n\x06STRING\x10\x08*y\n\nStatusCode\x12\x0c\n\x08NO_ERROR\x10\x00\x12\x11\n\rUNKNOWN_ERROR\x10\x01\x12\x11\n\rINVALID_PARAM\x10\x02\x12\x10\n\x0cINVALID_TYPE\x10\x03\x12\x11\n\rINVALID_VALUE\x10\x04\x12\x12\n\x0eREQUEST_FAILED\x10\x05'
  ,
  dependencies=[nanopb__pb2.DESCRIPTOR,])

_MSGID = _descriptor.EnumDescriptor(
  name='MsgId',
  full_name='MsgId',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MSG_ACK_NACK', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_PING_CMD', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_TERMINAL', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_SYS_TICK', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_SYS_INFO', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_PARAM_IO', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_SYS_CTRL', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=815,
  serialized_end=948,
)
_sym_db.RegisterEnumDescriptor(_MSGID)

MsgId = enum_type_wrapper.EnumTypeWrapper(_MSGID)
_SUBID = _descriptor.EnumDescriptor(
  name='SubId',
  full_name='SubId',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SUB_MSG_PARAM_IO_GET', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SUB_MSG_PARAM_IO_PUT', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SUB_MSG_PARAM_IO_SYNC', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SUB_MSG_PARAM_IO_LOAD', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SUB_MSG_SYS_CTRL_RESET', index=4, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=b'\020\001',
  serialized_start=951,
  serialized_end=1096,
)
_sym_db.RegisterEnumDescriptor(_SUBID)

SubId = enum_type_wrapper.EnumTypeWrapper(_SUBID)
_PARAMID = _descriptor.EnumDescriptor(
  name='ParamId',
  full_name='ParamId',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PARAM_INVALID', index=0, number=-1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_BOOT_COUNT', index=1, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_HW_VERSION', index=2, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_SW_VERSION', index=3, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_DEVICE_ID', index=4, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_BOARD_NAME', index=5, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_DESCRIPTION', index=6, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_SERIAL_NUMBER', index=7, number=10,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_DISK_UPDATE_RATE_MS', index=8, number=11,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_ACTIVITY_LED_SCALER', index=9, number=12,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAM_BOOT_MODE', index=10, number=13,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1099,
  serialized_end=1376,
)
_sym_db.RegisterEnumDescriptor(_PARAMID)

ParamId = enum_type_wrapper.EnumTypeWrapper(_PARAMID)
_PARAMTYPE = _descriptor.EnumDescriptor(
  name='ParamType',
  full_name='ParamType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BOOL', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UINT8', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UINT16', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UINT32', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FLOAT', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DOUBLE', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BYTES', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STRING', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1378,
  serialized_end=1493,
)
_sym_db.RegisterEnumDescriptor(_PARAMTYPE)

ParamType = enum_type_wrapper.EnumTypeWrapper(_PARAMTYPE)
_STATUSCODE = _descriptor.EnumDescriptor(
  name='StatusCode',
  full_name='StatusCode',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_ERROR', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_ERROR', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INVALID_PARAM', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INVALID_TYPE', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INVALID_VALUE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='REQUEST_FAILED', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1495,
  serialized_end=1616,
)
_sym_db.RegisterEnumDescriptor(_STATUSCODE)

StatusCode = enum_type_wrapper.EnumTypeWrapper(_STATUSCODE)
MSG_ACK_NACK = 0
MSG_PING_CMD = 1
MSG_TERMINAL = 2
MSG_SYS_TICK = 3
MSG_SYS_INFO = 4
MSG_PARAM_IO = 5
MSG_SYS_CTRL = 6
SUB_MSG_PARAM_IO_GET = 0
SUB_MSG_PARAM_IO_PUT = 1
SUB_MSG_PARAM_IO_SYNC = 2
SUB_MSG_PARAM_IO_LOAD = 3
SUB_MSG_SYS_CTRL_RESET = 0
PARAM_INVALID = -1
PARAM_BOOT_COUNT = 0
PARAM_HW_VERSION = 1
PARAM_SW_VERSION = 2
PARAM_DEVICE_ID = 3
PARAM_BOARD_NAME = 4
PARAM_DESCRIPTION = 5
PARAM_SERIAL_NUMBER = 10
PARAM_DISK_UPDATE_RATE_MS = 11
PARAM_ACTIVITY_LED_SCALER = 12
PARAM_BOOT_MODE = 13
UNKNOWN = 0
BOOL = 1
UINT8 = 2
UINT16 = 3
UINT32 = 4
FLOAT = 5
DOUBLE = 6
BYTES = 7
STRING = 8
NO_ERROR = 0
UNKNOWN_ERROR = 1
INVALID_PARAM = 2
INVALID_TYPE = 3
INVALID_VALUE = 4
REQUEST_FAILED = 5



_HEADER = _descriptor.Descriptor(
  name='Header',
  full_name='Header',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='msgId', full_name='Header.msgId', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='subId', full_name='Header.subId', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='uuid', full_name='Header.uuid', index=2,
      number=3, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\020', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=40,
  serialized_end=113,
)


_BASEMESSAGE = _descriptor.Descriptor(
  name='BaseMessage',
  full_name='BaseMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='BaseMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
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
  serialized_start=115,
  serialized_end=153,
)


_ACKNACKMESSAGE = _descriptor.Descriptor(
  name='AckNackMessage',
  full_name='AckNackMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='AckNackMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='acknowledge', full_name='AckNackMessage.acknowledge', index=1,
      number=2, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='status_code', full_name='AckNackMessage.status_code', index=2,
      number=3, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=155,
  serialized_end=258,
)


_PINGMESSAGE = _descriptor.Descriptor(
  name='PingMessage',
  full_name='PingMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='PingMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
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
  serialized_start=260,
  serialized_end=298,
)


_SYSTEMTICK = _descriptor.Descriptor(
  name='SystemTick',
  full_name='SystemTick',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='SystemTick.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='tick', full_name='SystemTick.tick', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
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
  serialized_start=300,
  serialized_end=351,
)


_CONSOLEMESSAGE = _descriptor.Descriptor(
  name='ConsoleMessage',
  full_name='ConsoleMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ConsoleMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='this_frame', full_name='ConsoleMessage.this_frame', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='total_frames', full_name='ConsoleMessage.total_frames', index=2,
      number=3, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='data', full_name='ConsoleMessage.data', index=3,
      number=4, type=12, cpp_type=9, label=2,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\003\010\200\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=353,
  serialized_end=472,
)


_SYSTEMINFOMESSAGE = _descriptor.Descriptor(
  name='SystemInfoMessage',
  full_name='SystemInfoMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='SystemInfoMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='systemTick', full_name='SystemInfoMessage.systemTick', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='swVersion', full_name='SystemInfoMessage.swVersion', index=2,
      number=3, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\002\010\020\222?\002x\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='description', full_name='SystemInfoMessage.description', index=3,
      number=4, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\002\010\020\222?\002x\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='serialNumber', full_name='SystemInfoMessage.serialNumber', index=4,
      number=5, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\002\010\020\222?\002x\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=475,
  serialized_end=637,
)


_PARAMIOMESSAGE = _descriptor.Descriptor(
  name='ParamIOMessage',
  full_name='ParamIOMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='ParamIOMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='id', full_name='ParamIOMessage.id', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\020', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='ParamIOMessage.type', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='data', full_name='ParamIOMessage.data', index=3,
      number=4, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\002\010@', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=639,
  serialized_end=763,
)


_SYSTEMCONTROLMESSAGE = _descriptor.Descriptor(
  name='SystemControlMessage',
  full_name='SystemControlMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='SystemControlMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
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
  serialized_start=765,
  serialized_end=812,
)

_BASEMESSAGE.fields_by_name['header'].message_type = _HEADER
_ACKNACKMESSAGE.fields_by_name['header'].message_type = _HEADER
_ACKNACKMESSAGE.fields_by_name['status_code'].enum_type = _STATUSCODE
_PINGMESSAGE.fields_by_name['header'].message_type = _HEADER
_SYSTEMTICK.fields_by_name['header'].message_type = _HEADER
_CONSOLEMESSAGE.fields_by_name['header'].message_type = _HEADER
_SYSTEMINFOMESSAGE.fields_by_name['header'].message_type = _HEADER
_PARAMIOMESSAGE.fields_by_name['header'].message_type = _HEADER
_PARAMIOMESSAGE.fields_by_name['id'].enum_type = _PARAMID
_PARAMIOMESSAGE.fields_by_name['type'].enum_type = _PARAMTYPE
_SYSTEMCONTROLMESSAGE.fields_by_name['header'].message_type = _HEADER
DESCRIPTOR.message_types_by_name['Header'] = _HEADER
DESCRIPTOR.message_types_by_name['BaseMessage'] = _BASEMESSAGE
DESCRIPTOR.message_types_by_name['AckNackMessage'] = _ACKNACKMESSAGE
DESCRIPTOR.message_types_by_name['PingMessage'] = _PINGMESSAGE
DESCRIPTOR.message_types_by_name['SystemTick'] = _SYSTEMTICK
DESCRIPTOR.message_types_by_name['ConsoleMessage'] = _CONSOLEMESSAGE
DESCRIPTOR.message_types_by_name['SystemInfoMessage'] = _SYSTEMINFOMESSAGE
DESCRIPTOR.message_types_by_name['ParamIOMessage'] = _PARAMIOMESSAGE
DESCRIPTOR.message_types_by_name['SystemControlMessage'] = _SYSTEMCONTROLMESSAGE
DESCRIPTOR.enum_types_by_name['MsgId'] = _MSGID
DESCRIPTOR.enum_types_by_name['SubId'] = _SUBID
DESCRIPTOR.enum_types_by_name['ParamId'] = _PARAMID
DESCRIPTOR.enum_types_by_name['ParamType'] = _PARAMTYPE
DESCRIPTOR.enum_types_by_name['StatusCode'] = _STATUSCODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Header = _reflection.GeneratedProtocolMessageType('Header', (_message.Message,), {
  'DESCRIPTOR' : _HEADER,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:Header)
  })
_sym_db.RegisterMessage(Header)

BaseMessage = _reflection.GeneratedProtocolMessageType('BaseMessage', (_message.Message,), {
  'DESCRIPTOR' : _BASEMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:BaseMessage)
  })
_sym_db.RegisterMessage(BaseMessage)

AckNackMessage = _reflection.GeneratedProtocolMessageType('AckNackMessage', (_message.Message,), {
  'DESCRIPTOR' : _ACKNACKMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:AckNackMessage)
  })
_sym_db.RegisterMessage(AckNackMessage)

PingMessage = _reflection.GeneratedProtocolMessageType('PingMessage', (_message.Message,), {
  'DESCRIPTOR' : _PINGMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:PingMessage)
  })
_sym_db.RegisterMessage(PingMessage)

SystemTick = _reflection.GeneratedProtocolMessageType('SystemTick', (_message.Message,), {
  'DESCRIPTOR' : _SYSTEMTICK,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:SystemTick)
  })
_sym_db.RegisterMessage(SystemTick)

ConsoleMessage = _reflection.GeneratedProtocolMessageType('ConsoleMessage', (_message.Message,), {
  'DESCRIPTOR' : _CONSOLEMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:ConsoleMessage)
  })
_sym_db.RegisterMessage(ConsoleMessage)

SystemInfoMessage = _reflection.GeneratedProtocolMessageType('SystemInfoMessage', (_message.Message,), {
  'DESCRIPTOR' : _SYSTEMINFOMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:SystemInfoMessage)
  })
_sym_db.RegisterMessage(SystemInfoMessage)

ParamIOMessage = _reflection.GeneratedProtocolMessageType('ParamIOMessage', (_message.Message,), {
  'DESCRIPTOR' : _PARAMIOMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:ParamIOMessage)
  })
_sym_db.RegisterMessage(ParamIOMessage)

SystemControlMessage = _reflection.GeneratedProtocolMessageType('SystemControlMessage', (_message.Message,), {
  'DESCRIPTOR' : _SYSTEMCONTROLMESSAGE,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:SystemControlMessage)
  })
_sym_db.RegisterMessage(SystemControlMessage)


_SUBID._options = None
_HEADER.fields_by_name['msgId']._options = None
_HEADER.fields_by_name['subId']._options = None
_HEADER.fields_by_name['uuid']._options = None
_ACKNACKMESSAGE.fields_by_name['status_code']._options = None
_CONSOLEMESSAGE.fields_by_name['this_frame']._options = None
_CONSOLEMESSAGE.fields_by_name['total_frames']._options = None
_CONSOLEMESSAGE.fields_by_name['data']._options = None
_SYSTEMINFOMESSAGE.fields_by_name['swVersion']._options = None
_SYSTEMINFOMESSAGE.fields_by_name['description']._options = None
_SYSTEMINFOMESSAGE.fields_by_name['serialNumber']._options = None
_PARAMIOMESSAGE.fields_by_name['id']._options = None
_PARAMIOMESSAGE.fields_by_name['type']._options = None
_PARAMIOMESSAGE.fields_by_name['data']._options = None
# @@protoc_insertion_point(module_scope)
