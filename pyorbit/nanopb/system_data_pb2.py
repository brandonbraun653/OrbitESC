# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: system_data.proto
# Protobuf Python Version: 4.25.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2
import serial_interface_pb2 as serial__interface__pb2
import motor_control_pb2 as motor__control__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x11system_data.proto\x1a\x0cnanopb.proto\x1a\x16serial_interface.proto\x1a\x13motor_control.proto\":\n\x11SystemTickMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x0c\n\x04tick\x18\x02 \x02(\r\"w\n\x0e\x43onsoleMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x19\n\nthis_frame\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x1b\n\x0ctotal_frames\x18\x03 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x04\x64\x61ta\x18\x04 \x02(\x0c\x42\x06\x92?\x03\x08\x80\x01\"\x99\x01\n\x11SystemInfoMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x12\n\nsystemTick\x18\x02 \x02(\r\x12\x1a\n\tswVersion\x18\x03 \x02(\tB\x07\x92?\x04\x08\x10x\x01\x12\x1c\n\x0b\x64\x65scription\x18\x04 \x02(\tB\x07\x92?\x04\x08\x10x\x01\x12\x1d\n\x0cserialNumber\x18\x05 \x02(\tB\x07\x92?\x04\x08\x10x\x01\"k\n\x13SystemStatusMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x12\n\nsystemTick\x18\x02 \x02(\r\x12\'\n\x0emotorCtrlState\x18\x03 \x02(\x0e\x32\x0f.MotorCtrlState\"y\n\x11SystemDataMessage\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12 \n\x02id\x18\x02 \x02(\x0e\x32\r.SystemDataIdB\x05\x92?\x02\x38\x08\x12\x11\n\ttimestamp\x18\x03 \x02(\r\x12\x16\n\x07payload\x18\x04 \x01(\x0c\x42\x05\x92?\x02\x08 \"=\n\x17\x41\x44\x43PhaseCurrentsPayload\x12\n\n\x02ia\x18\x01 \x02(\x02\x12\n\n\x02ib\x18\x02 \x02(\x02\x12\n\n\x02ic\x18\x03 \x02(\x02\"=\n\x17\x41\x44\x43PhaseVoltagesPayload\x12\n\n\x02va\x18\x01 \x02(\x02\x12\n\n\x02vb\x18\x02 \x02(\x02\x12\n\n\x02vc\x18\x03 \x02(\x02\"^\n\x18\x41\x44\x43SystemVoltagesPayload\x12\r\n\x05v_mcu\x18\x01 \x02(\x02\x12\x11\n\tv_dc_link\x18\x02 \x02(\x02\x12\x0e\n\x06v_temp\x18\x03 \x02(\x02\x12\x10\n\x08v_isense\x18\x04 \x02(\x02*m\n\x0cSystemDataId\x12\x14\n\x10SYS_DATA_INVALID\x10\x00\x12\x16\n\x12\x41\x44\x43_PHASE_CURRENTS\x10\x01\x12\x16\n\x12\x41\x44\x43_PHASE_VOLTAGES\x10\x02\x12\x17\n\x13\x41\x44\x43_SYSTEM_VOLTAGES\x10\x03')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'system_data_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['this_frame']._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['this_frame']._serialized_options = b'\222?\0028\010'
  _globals['_CONSOLEMESSAGE'].fields_by_name['total_frames']._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['total_frames']._serialized_options = b'\222?\0028\010'
  _globals['_CONSOLEMESSAGE'].fields_by_name['data']._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['data']._serialized_options = b'\222?\003\010\200\001'
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['swVersion']._options = None
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['swVersion']._serialized_options = b'\222?\004\010\020x\001'
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['description']._options = None
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['description']._serialized_options = b'\222?\004\010\020x\001'
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['serialNumber']._options = None
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['serialNumber']._serialized_options = b'\222?\004\010\020x\001'
  _globals['_SYSTEMDATAMESSAGE'].fields_by_name['id']._options = None
  _globals['_SYSTEMDATAMESSAGE'].fields_by_name['id']._serialized_options = b'\222?\0028\010'
  _globals['_SYSTEMDATAMESSAGE'].fields_by_name['payload']._options = None
  _globals['_SYSTEMDATAMESSAGE'].fields_by_name['payload']._serialized_options = b'\222?\002\010 '
  _globals['_SYSTEMDATAID']._serialized_start=871
  _globals['_SYSTEMDATAID']._serialized_end=980
  _globals['_SYSTEMTICKMESSAGE']._serialized_start=80
  _globals['_SYSTEMTICKMESSAGE']._serialized_end=138
  _globals['_CONSOLEMESSAGE']._serialized_start=140
  _globals['_CONSOLEMESSAGE']._serialized_end=259
  _globals['_SYSTEMINFOMESSAGE']._serialized_start=262
  _globals['_SYSTEMINFOMESSAGE']._serialized_end=415
  _globals['_SYSTEMSTATUSMESSAGE']._serialized_start=417
  _globals['_SYSTEMSTATUSMESSAGE']._serialized_end=524
  _globals['_SYSTEMDATAMESSAGE']._serialized_start=526
  _globals['_SYSTEMDATAMESSAGE']._serialized_end=647
  _globals['_ADCPHASECURRENTSPAYLOAD']._serialized_start=649
  _globals['_ADCPHASECURRENTSPAYLOAD']._serialized_end=710
  _globals['_ADCPHASEVOLTAGESPAYLOAD']._serialized_start=712
  _globals['_ADCPHASEVOLTAGESPAYLOAD']._serialized_end=773
  _globals['_ADCSYSTEMVOLTAGESPAYLOAD']._serialized_start=775
  _globals['_ADCSYSTEMVOLTAGESPAYLOAD']._serialized_end=869
# @@protoc_insertion_point(module_scope)
