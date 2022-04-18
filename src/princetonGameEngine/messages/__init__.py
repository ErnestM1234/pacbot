from enum import Enum
from .pacmanState_pb2 import PacmanState
from .lightState_pb2 import LightState
from .pacmanCommand_pb2 import PacmanCommand

#     remember brew install protobuf

class MsgType(Enum):
    LIGHT_STATE = 0
    PACMAN_LOCATION = 1
    PACMAN_COMMAND = 2
    FULL_STATE = 3

message_buffers = {
    MsgType.PACMAN_COMMAND: PacmanCommand,
    MsgType.FULL_STATE: PacmanState,
    MsgType.PACMAN_LOCATION: PacmanState.AgentState,
    MsgType.LIGHT_STATE: LightState
}


__all__ = ['MsgType', 'message_buffers', 'PacmanState', 'LightState',
            'PacmanCommand']
