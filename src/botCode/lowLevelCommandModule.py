#!/usr/bin/env python3

import os
import time
import robomodules as rm
import variables as var
from grid import grid
from low_level.motorsTesting import ArduinoMotors
from messages import MsgType, message_buffers, LightState, PacmanCommand

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 60

class LowLevelCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_COMMAND]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.current_command = None
        self.current_location = (13, 7) # starting location
        self.bot = ArduinoMotors()
        self.current_dir = PacmanCommand.EAST
        
    def _should_turn_left(self, cmd):
        return \
            (self.current_dir == PacmanCommand.NORTH and cmd  == PacmanCommand.WEST) or \
            (self.current_dir == PacmanCommand.SOUTH and cmd  == PacmanCommand.EAST) or \
            (self.current_dir == PacmanCommand.EAST and cmd  == PacmanCommand.NORTH) or \
            (self.current_dir == PacmanCommand.WEST and cmd  == PacmanCommand.SOUTH)

    def _should_turn_right(self, cmd):
        return \
            (self.current_dir == PacmanCommand.NORTH and cmd  == PacmanCommand.EAST) or \
            (self.current_dir == PacmanCommand.SOUTH and cmd  == PacmanCommand.WEST) or \
            (self.current_dir == PacmanCommand.EAST and cmd  == PacmanCommand.SOUTH) or \
            (self.current_dir == PacmanCommand.WEST and cmd  == PacmanCommand.NORTH)

    def _should_reverse(self, cmd):
        return \
            (self.current_dir == PacmanCommand.NORTH and cmd  == PacmanCommand.SOUTH) or \
            (self.current_dir == PacmanCommand.SOUTH and cmd  == PacmanCommand.NORTH) or \
            (self.current_dir == PacmanCommand.EAST and cmd  == PacmanCommand.WEST) or \
            (self.current_dir == PacmanCommand.WEST and cmd  == PacmanCommand.EAST)

    def _move_forward(self):
        self.bot.move_cells(1)

    def _turn_right(self):
        self.bot.turn_right() # change physical direction
        if self.current_dir == PacmanCommand.EAST: # update inner state of robot accordingly
            self.current_dir = PacmanCommand.SOUTH
        elif self.current_dir == PacmanCommand.SOUTH:
            self.current_dir = PacmanCommand.WEST
        elif self.current_dir == PacmanCommand.WEST:
            self.current_dir = PacmanCommand.NORTH
        else:
            self.current_dir = PacmanCommand.EAST
            
    def _turn_left(self):
        self.bot.turn_left()
        if self.current_dir == PacmanCommand.EAST:
            self.current_dir = PacmanCommand.NORTH
        elif self.current_dir == PacmanCommand.NORTH:
            self.current_dir = PacmanCommand.WEST
        elif self.current_dir == PacmanCommand.WEST:
            self.current_dir = PacmanCommand.SOUTH
        else:
            self.current_dir = PacmanCommand.EAST

    def _turn_around(self):
        #TODO: Turn 180 degrees
        if self.current_dir == PacmanCommand.EAST:
            self.current_dir = PacmanCommand.WEST
        elif self.current_dir == PacmanCommand.NORTH:
            self.current_dir = PacmanCommand.SOUTH
        elif self.current_dir == PacmanCommand.WEST:
            self.current_dir = PacmanCommand.EAST
        else:
            self.current_dir = PacmanCommand.NORTH
    
    def _reverse(self):
        self.bot.move_backwards()

    def _execute_command(self):
        if self.current_command:
            cmd = self.current_command
            self.current_command = None

            if cmd == PacmanCommand.STOP:               # check stop
                self.bot.stop()
                return

            if self._should_turn_left(cmd):             # follow direction
                self._turn_left()
            elif self._should_turn_right(cmd):
                self._turn_right()
            if self._should_reverse(cmd):
                self._reverse()
            else:
                self._move_forward()
            
            if cmd == PacmanCommand.EAST:               # update virtual position
                self.current_location = (self.current_location[0] + 1, self.current_location[1])
            elif cmd == PacmanCommand.WEST:
                self.current_location = (self.current_location[0] - 1, self.current_location[1])
            elif cmd == PacmanCommand.NORTH:
                self.current_location = (self.current_location[0], self.current_location[1] + 1)
            elif cmd == PacmanCommand.SOUTH:
                self.current_location = (self.current_location[0], self.current_location[1] - 1)

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_COMMAND:
            self.current_command = msg.dir

    def tick(self):
        if self.current_command:
            self._execute_command()
            time.sleep(1) # todo: remove this later

    def kill(self):
        self.bot.stop()


def main():
    module = LowLevelCommandModule(ADDRESS, PORT)
    try:
        module.run()
    except KeyboardInterrupt:
        module.kill()
if __name__ == "__main__":
    main()
