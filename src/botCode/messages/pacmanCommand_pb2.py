# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pacmanCommand.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13pacmanCommand.proto\x12\x07\x62otCode\"\x7f\n\rPacmanCommand\x12-\n\x03\x64ir\x18\x01 \x02(\x0e\x32 .botCode.PacmanCommand.Direction\"?\n\tDirection\x12\x08\n\x04STOP\x10\x01\x12\t\n\x05NORTH\x10\x02\x12\t\n\x05SOUTH\x10\x03\x12\x08\n\x04\x45\x41ST\x10\x04\x12\x08\n\x04WEST\x10\x05')



_PACMANCOMMAND = DESCRIPTOR.message_types_by_name['PacmanCommand']
_PACMANCOMMAND_DIRECTION = _PACMANCOMMAND.enum_types_by_name['Direction']
PacmanCommand = _reflection.GeneratedProtocolMessageType('PacmanCommand', (_message.Message,), {
  'DESCRIPTOR' : _PACMANCOMMAND,
  '__module__' : 'pacmanCommand_pb2'
  # @@protoc_insertion_point(class_scope:botCode.PacmanCommand)
  })
_sym_db.RegisterMessage(PacmanCommand)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _PACMANCOMMAND._serialized_start=32
  _PACMANCOMMAND._serialized_end=159
  _PACMANCOMMAND_DIRECTION._serialized_start=96
  _PACMANCOMMAND_DIRECTION._serialized_end=159
# @@protoc_insertion_point(module_scope)
