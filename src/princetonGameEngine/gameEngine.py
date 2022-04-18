#!/usr/bin/env python3

import os, sys, logging
import time
import robomodules as rm
from messages import *
from pacbot.variables import game_frequency, ticks_per_update
from pacbot import StateConverter, GameState

ADDRESS = os.environ.get("BIND_ADDRESS","localhost") # the address of the game engine server
# ADDRESS = os.environ.get("BIND_ADDRESS","192.168.1.55")
PORT = os.environ.get("BIND_PORT", 11293)            # the port the game engine server is listening to

FREQUENCY = game_frequency * ticks_per_update

class GameEngine(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_LOCATION]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.loop.add_reader(sys.stdin, self.keypress)

        self.game = GameState()

    def _write_state(self):
        full_state = StateConverter.convert_game_state_to_full(self.game)
        self.write(full_state.SerializeToString(), MsgType.FULL_STATE)

        light_state = StateConverter.convert_game_state_to_light(self.game)
        self.write(light_state.SerializeToString(), MsgType.LIGHT_STATE)


    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_LOCATION:
            self.game.pacbot.update((msg.x, msg.y))

    def tick(self):
        # this function will get called in a loop with FREQUENCY frequency
        if self.game.play:
            # This will become asynchronous
            self.game.next_step()
        self._write_state()

    def keypress(self):
        char = sys.stdin.read(1)
        # For some reason I couldn't quite get this to do what I wanted
        # Still it's a bit cleaner than otherwise
        sys.stdout.write("\033[F")
        sys.stdout.write("\033[K")
        sys.stdout.flush()
        if char == "r":
            logging.info("Restarting...")
            self.game.restart()
            self._write_state()
        elif char == "n":
            # how to write a command
            new_msg = PacmanCommand()
            new_msg.dir = PacmanCommand.NORTH
            self.write(new_msg.SerializeToString(), MsgType.PACMAN_COMMAND)
            print("command: North")
        elif char == "s":
            # how to write a command
            new_msg = PacmanCommand()
            new_msg.dir = PacmanCommand.SOUTH
            self.write(new_msg.SerializeToString(), MsgType.PACMAN_COMMAND)
            print("command: South")
        elif char == "e":
            # how to write a command
            new_msg = PacmanCommand()
            new_msg.dir = PacmanCommand.EAST
            self.write(new_msg.SerializeToString(), MsgType.PACMAN_COMMAND)
            print("command: East")
        elif char == "w":
            # how to write a command
            new_msg = PacmanCommand()
            new_msg.dir = PacmanCommand.WEST
            self.write(new_msg.SerializeToString(), MsgType.PACMAN_COMMAND)
            print("command: West")
        elif char == "p":
            if (self.game.play):
                logging.info('Game is paused')
                self.game.pause()
            else:
                logging.info('Game resumed')
                self.game.unpause()
        elif char == "q":
            logging.info("Quitting...")
            self.quit() 

def main():
    # logger automatically adds timestamps
    # I wanted it to print each sequentially but it did not want to
    logging.basicConfig(level=logging.INFO, format='%(asctime)s: %(message)s',
                        datefmt="%I:%M:%S %p")
    engine = GameEngine(ADDRESS, PORT)
    print('Game is paused.')
    print('Controls:')
    print('    r - restart')
    print('    p - (un)pause')
    print('    q - quit')
    print('------------')
    print('    n - north')
    print('    s - south')
    print('    e - east')
    print('    w - west')

    engine.run()


if __name__ == "__main__":
    main()
