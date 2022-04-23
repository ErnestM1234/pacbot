#!/usr/bin/env python3

"""
[
    (L | R | F, -1 | FORWARDS_DIST),
    (L | R | F, -1 | FORWARDS_DIST),
    (L | R | F, -1 | FORWARDS_DIST),
    (L | R | F, -1 | FORWARDS_DIST),
    (L | R | F, -1 | FORWARDS_DIST),
]


"""

import os, sys, logging
import time
import robomodules as rm
from messages import *
from pacbot.variables import game_frequency, ticks_per_update
from pacbot import StateConverter, GameState

ADDRESS = os.environ.get("BIND_ADDRESS","localhost") # the address of the game engine server
# ADDRESS = os.environ.get("BIND_ADDRESS","172.20.10.3")
PORT = os.environ.get("BIND_PORT", 11293)            # the port the game engine server is listening to

FREQUENCY = game_frequency * ticks_per_update

class GameEngine(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.loop.add_reader(sys.stdin, self.keypress)

        self.game = GameState()

    """ command_north()
    input: void
    output: void
    send command for pacbot to go north
    """
    def command_north(self):
        command = PacmanCommand()
        command.dir = PacmanCommand.NORTH
        self.write(command.SerializeToString(), MsgType.PACMAN_COMMAND)
        print("command: North")
    
    """ command_south()
    input: void
    output: void
    send command for pacbot to go south
    """
    def command_south(self):
        command = PacmanCommand()
        command.dir = PacmanCommand.SOUTH
        self.write(command.SerializeToString(), MsgType.PACMAN_COMMAND)
        print("command: South")

    """ command_east()
    input: void
    output: void
    send command for pacbot to go east
    """
    def command_east(self):
        command = PacmanCommand()
        command.dir = PacmanCommand.EAST
        self.write(command.SerializeToString(), MsgType.PACMAN_COMMAND)
        print("command: East")

    """ command_west()
    input: void
    output: void
    send command for pacbot to go west
    """
    def command_west(self):
        command = PacmanCommand()
        command.dir = PacmanCommand.WEST
        self.write(command.SerializeToString(), MsgType.PACMAN_COMMAND)
        print("command: West")
    
    """ command_stop()
    input: void
    output: void
    send command for pacbot to stop
    """
    def command_stop(self):
        command = PacmanCommand()
        command.dir = PacmanCommand.STOP
        self.write(command.SerializeToString(), MsgType.PACMAN_COMMAND)
        print("command: stop")

    def _write_state(self):
        # full_state = StateConverter.convert_game_state_to_full(self.game)
        # self.write(full_state.SerializeToString(), MsgType.FULL_STATE)

        # light_state = StateConverter.convert_game_state_to_light(self.game)
        # self.write(light_state.SerializeToString(), MsgType.LIGHT_STATE)
        return


    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.LIGHT_STATE:
            # todo: update game engine
            self.gameEngine.update(msg)

            return
            # self.game.pacbot.update((msg.x, msg.y))

    def tick(self):
        # this function will get called in a loop with FREQUENCY frequency
        self._write_state()

        if self.game.play:
            return
            # call game engine
            # self.gameEngine.getAction()

            # This will become asynchronous
            # self.game.next_step()
            

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
            self.command_north()
        elif char == "s":
            self.command_south()
        elif char == "e":
            self.command_east()
        elif char == "w":
            self.command_west()
        elif char == "s":
            self.command_stop()
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
