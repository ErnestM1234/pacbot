#!/usr/bin/env python3

import os
import robomodules as rm
from messages import *

SERVER_ADDRESS = os.environ.get("BIND_ADDRESS","localhost")
# SERVER_ADDRESS = os.environ.get("BIND_ADDRESS","172.20.10.3")
SERVER_PORT = os.environ.get("BIND_PORT", 11297)

LOCAL_ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost") # always on local host
LOCAL_PORT = os.environ.get("LOCAL_PORT", 11295)

# GAME_ENGINE_ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
# GAME_ENGINE_ADDRESS = os.environ.get("LOCAL_ADDRESS","10.24.106.144")
# GAME_ENGINE_PORT = os.environ.get("LOCAL_PORT", 11293)

SERVER_FREQUENCY = 0
GAME_ENGINE_FREQUENCY = 0 # no idea what this value means
LOCAL_FREQUENCY = 30

"""
Vocab (please read!):

SERVER      - Harvard's server that gives pacbot information about the state
            of the game and where pac bot is
LOCAL       - Running locally on the pac bot and controls motors
GAME_ENGINE - Princeton's computation server that gives pacbot actions
 remember brew install protobuf
"""


# this connects to the Server
class PacbotServerClient(rm.ProtoModule):
    def __init__(self, addr, port, loop):
        self.subscriptions = [MsgType.LIGHT_STATE]
        # this connects to the game engine
        super().__init__(addr, port, message_buffers, MsgType, SERVER_FREQUENCY, self.subscriptions, loop)
        self.state = None

    def msg_received(self, msg, msg_type):
        # This gets called whenever any message is received
        # This module will connect to server and receive the game state
        if msg_type == MsgType.LIGHT_STATE:
            self.state = msg

    def tick(self):
        return

    def get_state(self):
        return self.state

# this connects to the pacbot('s sever)
class PacbotServerCommsModule(rm.ProtoModule):
    def __init__(self, server_addr, server_port, local_addr, local_port):
        self.subscriptions = [MsgType.ACK]
        # this connects to the local server
        super().__init__(local_addr, local_port, message_buffers, MsgType, LOCAL_FREQUENCY, self.subscriptions)
        # this connects to the server
        self.server_module = PacbotServerClient(server_addr, server_port, self.loop)
        self.server_module.connect()
        
    def msg_received(self, msg, msg_type):
        # This gets called whenever any message is received
        if msg_type == MsgType.ACK:
            state = self.server_module.get_state()
            if state != None:
                # Broadcast state to local modules
                self.write(state.SerializeToString(), MsgType.LIGHT_STATE)

    def tick(self):
        return
        # # Get state from the server
        # state = self.server_module.get_state()
        # if state != None:
        #     # Broadcast state to local modules
        #     self.write(state.SerializeToString(), MsgType.LIGHT_STATE)

def main():
    module = PacbotServerCommsModule(SERVER_ADDRESS, SERVER_PORT, LOCAL_ADDRESS, LOCAL_PORT)
    module.run()

if __name__ == "__main__":
    main()
