# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: serial_interface.proto
# Protobuf Python Version: 4.25.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x16serial_interface.proto\x1a\x0cnanopb.proto\"I\n\x06Header\x12\x14\n\x05msgId\x18\x01 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05subId\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x13\n\x04uuid\x18\x03 \x02(\rB\x05\x92?\x02\x38\x10\"&\n\x0b\x42\x61seMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\"|\n\x0e\x41\x63kNackMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x13\n\x0b\x61\x63knowledge\x18\x02 \x02(\x08\x12\'\n\x0bstatus_code\x18\x03 \x02(\x0e\x32\x0b.StatusCodeB\x05\x92?\x02\x38\x08\x12\x13\n\x04\x64\x61ta\x18\x04 \x01(\x0c\x42\x05\x92?\x02\x08@\"&\n\x0bPingMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header*\xab\x01\n\x05MsgId\x12\x10\n\x0cMSG_ACK_NACK\x10\x00\x12\x10\n\x0cMSG_PING_CMD\x10\x01\x12\x10\n\x0cMSG_TERMINAL\x10\x02\x12\x10\n\x0cMSG_SYS_TICK\x10\x03\x12\x10\n\x0cMSG_SYS_INFO\x10\x04\x12\x10\n\x0cMSG_PARAM_IO\x10\x05\x12\x10\n\x0cMSG_SYS_CTRL\x10\x06\x12\x10\n\x0cMSG_SYS_DATA\x10\x07\x12\x12\n\x0eMSG_SYS_STATUS\x10\x08*\x19\n\x05SubId\x12\x10\n\x0cSUB_MSG_NONE\x10\x00*y\n\nStatusCode\x12\x0c\n\x08NO_ERROR\x10\x00\x12\x11\n\rUNKNOWN_ERROR\x10\x01\x12\x11\n\rINVALID_PARAM\x10\x02\x12\x10\n\x0cINVALID_TYPE\x10\x03\x12\x11\n\rINVALID_VALUE\x10\x04\x12\x12\n\x0eREQUEST_FAILED\x10\x05')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'serial_interface_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_HEADER'].fields_by_name['msgId']._options = None
  _globals['_HEADER'].fields_by_name['msgId']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['subId']._options = None
  _globals['_HEADER'].fields_by_name['subId']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['uuid']._options = None
  _globals['_HEADER'].fields_by_name['uuid']._serialized_options = b'\222?\0028\020'
  _globals['_ACKNACKMESSAGE'].fields_by_name['status_code']._options = None
  _globals['_ACKNACKMESSAGE'].fields_by_name['status_code']._serialized_options = b'\222?\0028\010'
  _globals['_ACKNACKMESSAGE'].fields_by_name['data']._options = None
  _globals['_ACKNACKMESSAGE'].fields_by_name['data']._serialized_options = b'\222?\002\010@'
  _globals['_MSGID']._serialized_start=322
  _globals['_MSGID']._serialized_end=493
  _globals['_SUBID']._serialized_start=495
  _globals['_SUBID']._serialized_end=520
  _globals['_STATUSCODE']._serialized_start=522
  _globals['_STATUSCODE']._serialized_end=643
  _globals['_HEADER']._serialized_start=40
  _globals['_HEADER']._serialized_end=113
  _globals['_BASEMESSAGE']._serialized_start=115
  _globals['_BASEMESSAGE']._serialized_end=153
  _globals['_ACKNACKMESSAGE']._serialized_start=155
  _globals['_ACKNACKMESSAGE']._serialized_end=279
  _globals['_PINGMESSAGE']._serialized_start=281
  _globals['_PINGMESSAGE']._serialized_end=319
# @@protoc_insertion_point(module_scope)
