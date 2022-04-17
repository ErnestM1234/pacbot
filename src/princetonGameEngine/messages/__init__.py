from enum import Enum
from .pacmanState_pb2 import PacmanState
from .lightState_pb2 import LightState
from .pacmanCommand_pb2 import PacmanCommand

class MsgType(Enum):
    LIGHT_STATE = 0
    PACMAN_LOCATION = 1
    FULL_STATE = 2
    PACMAN_COMMAND = 3

message_buffers = {
    MsgType.PACMAN_COMMAND: PacmanCommand,
    MsgType.FULL_STATE: PacmanState,
    MsgType.PACMAN_LOCATION: PacmanState.AgentState,
    MsgType.LIGHT_STATE: LightState
}


__all__ = ['MsgType', 'message_buffers', 'PacmanState', 'LightState',
            'PacmanCommand']
