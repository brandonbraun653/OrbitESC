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
  serialized_pb=b'\n\x16serial_interface.proto\x1a\x0cnanopb.proto\"I\n\x06Header\x12\x14\n\x05msgId\x18\x01 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05subId\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x13\n\x04uuid\x18\x03 \x02(\rB\x05\x92?\x02\x38\x10\"&\n\x0b\x42\x61seMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\"|\n\x0e\x41\x63kNackMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x13\n\x0b\x61\x63knowledge\x18\x02 \x02(\x08\x12\'\n\x0bstatus_code\x18\x03 \x02(\x0e\x32\x0b.StatusCodeB\x05\x92?\x02\x38\x08\x12\x13\n\x04\x64\x61ta\x18\x04 \x01(\x0c\x42\x05\x92?\x02\x08@\"&\n\x0bPingMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header*\xab\x01\n\x05MsgId\x12\x10\n\x0cMSG_ACK_NACK\x10\x00\x12\x10\n\x0cMSG_PING_CMD\x10\x01\x12\x10\n\x0cMSG_TERMINAL\x10\x02\x12\x10\n\x0cMSG_SYS_TICK\x10\x03\x12\x10\n\x0cMSG_SYS_INFO\x10\x04\x12\x10\n\x0cMSG_PARAM_IO\x10\x05\x12\x10\n\x0cMSG_SYS_CTRL\x10\x06\x12\x10\n\x0cMSG_SYS_DATA\x10\x07\x12\x12\n\x0eMSG_SYS_STATUS\x10\x08*\x19\n\x05SubId\x12\x10\n\x0cSUB_MSG_NONE\x10\x00*y\n\nStatusCode\x12\x0c\n\x08NO_ERROR\x10\x00\x12\x11\n\rUNKNOWN_ERROR\x10\x01\x12\x11\n\rINVALID_PARAM\x10\x02\x12\x10\n\x0cINVALID_TYPE\x10\x03\x12\x11\n\rINVALID_VALUE\x10\x04\x12\x12\n\x0eREQUEST_FAILED\x10\x05'
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
    _descriptor.EnumValueDescriptor(
      name='MSG_SYS_DATA', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MSG_SYS_STATUS', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=322,
  serialized_end=493,
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
      name='SUB_MSG_NONE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=495,
  serialized_end=520,
)
_sym_db.RegisterEnumDescriptor(_SUBID)

SubId = enum_type_wrapper.EnumTypeWrapper(_SUBID)
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
  serialized_start=522,
  serialized_end=643,
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
MSG_SYS_DATA = 7
MSG_SYS_STATUS = 8
SUB_MSG_NONE = 0
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
    _descriptor.FieldDescriptor(
      name='data', full_name='AckNackMessage.data', index=3,
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
  serialized_start=155,
  serialized_end=279,
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
  serialized_start=281,
  serialized_end=319,
)

_BASEMESSAGE.fields_by_name['header'].message_type = _HEADER
_ACKNACKMESSAGE.fields_by_name['header'].message_type = _HEADER
_ACKNACKMESSAGE.fields_by_name['status_code'].enum_type = _STATUSCODE
_PINGMESSAGE.fields_by_name['header'].message_type = _HEADER
DESCRIPTOR.message_types_by_name['Header'] = _HEADER
DESCRIPTOR.message_types_by_name['BaseMessage'] = _BASEMESSAGE
DESCRIPTOR.message_types_by_name['AckNackMessage'] = _ACKNACKMESSAGE
DESCRIPTOR.message_types_by_name['PingMessage'] = _PINGMESSAGE
DESCRIPTOR.enum_types_by_name['MsgId'] = _MSGID
DESCRIPTOR.enum_types_by_name['SubId'] = _SUBID
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


_HEADER.fields_by_name['msgId']._options = None
_HEADER.fields_by_name['subId']._options = None
_HEADER.fields_by_name['uuid']._options = None
_ACKNACKMESSAGE.fields_by_name['status_code']._options = None
_ACKNACKMESSAGE.fields_by_name['data']._options = None
# @@protoc_insertion_point(module_scope)
