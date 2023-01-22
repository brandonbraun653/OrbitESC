# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: serial_interface.proto

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
  serialized_pb=b'\n\x16serial_interface.proto\x1a\x0cnanopb.proto\"M\n\nInstHeader\x12\x14\n\x05msgId\x18\x01 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05subId\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x13\n\x04size\x18\x03 \x02(\rB\x05\x92?\x02\x38\x08\"*\n\x0b\x42\x61seMessage\x12\x1b\n\x06header\x18\x01 \x02(\x0b\x32\x0b.InstHeader\"B\n\x0e\x41\x63kNackMessage\x12\x1b\n\x06header\x18\x01 \x02(\x0b\x32\x0b.InstHeader\x12\x13\n\x0b\x61\x63knowledge\x18\x02 \x02(\x08\"*\n\x0bPingMessage\x12\x1b\n\x06header\x18\x01 \x02(\x0b\x32\x0b.InstHeader\"Y\n\x0e\x43onsoleMessage\x12\x1b\n\x06header\x18\x01 \x02(\x0b\x32\x0b.InstHeader\x12\x14\n\x05\x66rame\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x04\x64\x61ta\x18\x03 \x02(\x0c\x42\x06\x92?\x03\x08\x80\x01\"\xa6\x01\n\x11SystemInfoMessage\x12\x1b\n\x06header\x18\x01 \x02(\x0b\x32\x0b.InstHeader\x12\x12\n\nsystemTick\x18\x02 \x02(\r\x12\x1d\n\tswVersion\x18\x03 \x02(\tB\n\x92?\x02\x08\x10\x92?\x02x\x01\x12\x1f\n\x0b\x64\x65scription\x18\x04 \x02(\tB\n\x92?\x02\x08\x10\x92?\x02x\x01\x12 \n\x0cserialNumber\x18\x05 \x02(\tB\n\x92?\x02\x08\x10\x92?\x02x\x01'
  ,
  dependencies=[nanopb__pb2.DESCRIPTOR,])




_INSTHEADER = _descriptor.Descriptor(
  name='InstHeader',
  full_name='InstHeader',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='msgId', full_name='InstHeader.msgId', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='subId', full_name='InstHeader.subId', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='size', full_name='InstHeader.size', index=2,
      number=3, type=13, cpp_type=3, label=2,
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
  serialized_start=40,
  serialized_end=117,
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
  serialized_start=119,
  serialized_end=161,
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
  serialized_start=163,
  serialized_end=229,
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
  serialized_start=231,
  serialized_end=273,
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
      name='frame', full_name='ConsoleMessage.frame', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\0028\010', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='data', full_name='ConsoleMessage.data', index=2,
      number=3, type=12, cpp_type=9, label=2,
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
  serialized_start=275,
  serialized_end=364,
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
  serialized_start=367,
  serialized_end=533,
)

_BASEMESSAGE.fields_by_name['header'].message_type = _INSTHEADER
_ACKNACKMESSAGE.fields_by_name['header'].message_type = _INSTHEADER
_PINGMESSAGE.fields_by_name['header'].message_type = _INSTHEADER
_CONSOLEMESSAGE.fields_by_name['header'].message_type = _INSTHEADER
_SYSTEMINFOMESSAGE.fields_by_name['header'].message_type = _INSTHEADER
DESCRIPTOR.message_types_by_name['InstHeader'] = _INSTHEADER
DESCRIPTOR.message_types_by_name['BaseMessage'] = _BASEMESSAGE
DESCRIPTOR.message_types_by_name['AckNackMessage'] = _ACKNACKMESSAGE
DESCRIPTOR.message_types_by_name['PingMessage'] = _PINGMESSAGE
DESCRIPTOR.message_types_by_name['ConsoleMessage'] = _CONSOLEMESSAGE
DESCRIPTOR.message_types_by_name['SystemInfoMessage'] = _SYSTEMINFOMESSAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

InstHeader = _reflection.GeneratedProtocolMessageType('InstHeader', (_message.Message,), {
  'DESCRIPTOR' : _INSTHEADER,
  '__module__' : 'serial_interface_pb2'
  # @@protoc_insertion_point(class_scope:InstHeader)
  })
_sym_db.RegisterMessage(InstHeader)

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


_INSTHEADER.fields_by_name['msgId']._options = None
_INSTHEADER.fields_by_name['subId']._options = None
_INSTHEADER.fields_by_name['size']._options = None
_CONSOLEMESSAGE.fields_by_name['frame']._options = None
_CONSOLEMESSAGE.fields_by_name['data']._options = None
_SYSTEMINFOMESSAGE.fields_by_name['swVersion']._options = None
_SYSTEMINFOMESSAGE.fields_by_name['description']._options = None
_SYSTEMINFOMESSAGE.fields_by_name['serialNumber']._options = None
# @@protoc_insertion_point(module_scope)
