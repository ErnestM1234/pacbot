#!/usr/bin/env python3

import os
import robomodules as rm
from messages import *

SERVER_ADDRESS = os.environ.get("BIND_ADDRESS","localhost")
# SERVER_ADDRESS = os.environ.get("BIND_ADDRESS","192.168.1.55")
SERVER_PORT = os.environ.get("BIND_PORT", 11297)

LOCAL_ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost") # always on local host
LOCAL_PORT = os.environ.get("LOCAL_PORT", 11295)

GAME_ENGINE_ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
#LOCAL_ADDRESS = os.environ.get("LOCAL_ADDRESS","192.168.1.55") # some other IP
GAME_ENGINE_PORT = os.environ.get("LOCAL_PORT", 11293)

SERVER_FREQUENCY = 0
GAME_ENGINE_FREQUENCY = 0 # no idea what this value means
LOCAL_FREQUENCY = 30

"""
Vocab (please read!):

SERVER      - Harvard's server that gives pacbot information about the state
            of the game and where pac bot is
LOCAL       - Running locally on the pac bot and controls motors
GAME_ENGINE - Princeton's computation server that gives pacbot actions
    
"""


# this connects to the Game Engine
# for silly reasons these classes have to be nested
class PacbotGameEngineClient(rm.ProtoModule):
    def __init__(self, addr, port, loop):
        self.subscriptions = [MsgType.PACMAN_COMMAND]
        # this connects to the game engine
        super().__init__(addr, port, message_buffers, MsgType, GAME_ENGINE_FREQUENCY, self.subscriptions, loop)
        self.command = None

    def msg_received(self, msg, msg_type):
        # This gets called whenever any message is received
        # This module will connect to server and receive the game state
        if msg_type == MsgType.PACMAN_COMMAND:
            self.command = msg

    def tick(self):
        return        
        
    def get_command(self):
        return self.command


class PacbotGameEngineCommsModule(rm.ProtoModule):
    def __init__(self, game_engine_addr, game_engine_port, local_addr, local_port):
        self.subscriptions = [MsgType.PACMAN_LOCATION]
        # this connects to the local server
        super().__init__(local_addr, local_port, message_buffers, MsgType, LOCAL_FREQUENCY, self.subscriptions)
        # this connects to the game engine
        self.game_engine_module = PacbotGameEngineClient(game_engine_addr, game_engine_port, self.loop)
        self.game_engine_module.connect()

    def msg_received(self, msg, msg_type):
        # broad cast information received from pacbot state to the game engine
        if msg_type == MsgType.PACMAN_LOCATION:
           self.game_engine_module.write(msg.SerializeToString(), MsgType.PACMAN_LOCATION)
        return

    def tick(self):
        # Get command from the game engine
        command = self.game_engine_module.get_command()
        if command != None:
            # Broadcast commands to local modules
           self.write(command.SerializeToString(), MsgType.PACMAN_COMMAND)

def main():
    module = PacbotGameEngineCommsModule(GAME_ENGINE_ADDRESS, GAME_ENGINE_PORT, LOCAL_ADDRESS, LOCAL_PORT)
    module.run()

if __name__ == "__main__":
    main()
