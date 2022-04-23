from enum import Enum
from .pacmanState_pb2 import PacmanState
from .lightState_pb2 import LightState
from .pacmanCommand_pb2 import PacmanCommand
from .test_pb2 import Test
from .pacCommand_pb2 import PacCommand

#     remember brew install protobuf

class MsgType(Enum):
    LIGHT_STATE = 0
    PACMAN_LOCATION = 1
    PACMAN_COMMAND = 2
    FULL_STATE = 3
    TEST = 4
    PAC_COMMAND = 5

message_buffers = {
    MsgType.PACMAN_COMMAND: PacmanCommand,
    MsgType.FULL_STATE: PacmanState,
    MsgType.PACMAN_LOCATION: PacmanState.AgentState,
    MsgType.LIGHT_STATE: LightState,
    MsgType.TEST: Test,
    MsgType.PAC_COMMAND: PacCommand
}


__all__ = ['MsgType', 'message_buffers', 'PacmanState', 'LightState',
            'PacmanCommand', 'Test', 'PacCommand',]
