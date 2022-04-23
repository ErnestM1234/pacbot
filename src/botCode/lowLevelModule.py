#!/usr/bin/env python3

import os
import time
import robomodules as rm
import variables as var
from grid import grid
from collections import deque
from messages import MsgType, message_buffers, LightState, PacmanCommand
from messages.ack_pb2 import Ack
from messages.pacCommand_pb2 import PacCommand
from low_level.sensorTesting import *

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)


# FORWARDS   = 0
# ROTATE     = 1
# FORCE_STOP = 2
FREQUENCY = 30

class LowLevelCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.arduino = ArduinoComms()
        self.subscriptions = [MsgType.PAC_COMMAND]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.command_queue = deque()
        self.current_command = (PacCommand.FORWARDS, 0) # in the form of a tuple
        self.pending_completion = False
        
    def _move_forward(self):
        self.arduino.write(FORWARDS, 1, False, False)

    def _turn_left(self):
        self.arduino.write(ROTATE, 0, True, False)
            
    def _turn_right(self):
        self.arduino.write(ROTATE, 0, False, True)

    def _execute_command(self):
        
        if not self.current_command:
            return False
          
        self.pending_completion = True        
        cmd = self.current_command
        self.current_command = None
        if (len(self.command_queue) > 0):
            self.current_command = self.command_queue.popleft()
        
        while not self.arduino.checkAck():
            print("test")
            if (cmd[0] == PacCommand.FORWARDS):
                self.arduino.write(FORWARDS, cmd[1], False, False)
            elif (cmd[0] == PacCommand.LEFT):
                self.arduino.write(ROTATE, 0, True, False)
            elif (cmd[0] == PacCommand.RIGHT):
                self.arduino.write(ROTATE, 0, False, True)
                
        self.pending_completion = False
        
        ack = Ack()
        ack.hasAck = 1
        self.write(ack.SerializeToString(), MsgType.ACK)
        return True
            
    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PAC_COMMAND:
            self.command_queue.append((msg.command.direction, msg.command.forwards_distance))
            # print(str(self.command_queue))
    
    def tick(self):
        if self.current_command and not self.pending_completion:
            self._execute_command()
            
    def kill(self):
        output = "{sto}"
        while True:
            self.arduino.write(output.encode('utf-8'))

def main():
    module = LowLevelCommandModule(ADDRESS, PORT)
    module.run()
    try:
        module.run()
    except KeyboardInterrupt:
        module.kill()
if __name__ == "__main__":
    main()