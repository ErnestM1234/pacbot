from enum import Enum
from .lightState_pb2 import LightState
from .pacmanState_pb2 import PacmanState
from .pacmanCommand_pb2 import PacmanCommand
from .test_pb2 import Test
from .pacCommand_pb2 import PacCommand
from .ack_pb2 import Ack


class MsgType(Enum):
    LIGHT_STATE = 0
    PACMAN_LOCATION = 1
    PACMAN_COMMAND = 2
    TEST = 3
    PAC_COMMAND = 4
    ACK = 5



message_buffers = {
    MsgType.LIGHT_STATE: LightState,
    MsgType.PACMAN_LOCATION: PacmanState.AgentState,
    MsgType.PACMAN_COMMAND: PacmanCommand,
    MsgType.TEST: Test,
    MsgType.PAC_COMMAND: PacCommand
    MsgType.ACK: Ack
}


__all__ = ['MsgType', 'message_buffers', 'LightState', 'PacmanState',
           'PacmanCommand', 'Test', 'PacCommand', 'Ack']
