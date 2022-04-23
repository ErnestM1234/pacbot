import os, sys, logging
import time
import robomodules as rm
from messages import *
from pacbot.variables import game_frequency, ticks_per_update
from messages.pacCommand_pb2 import PacCommand

FREQUENCY = game_frequency * ticks_per_update


# ADDRESS = os.environ.get("BIND_ADDRESS","localhost") # the address of the game engine server
ADDRESS = os.environ.get("BIND_ADDRESS","172.20.10.3")
PORT = os.environ.get("BIND_PORT", 11293)            # the port the game engine server is listening to


# state is a dict with keys:
#    pellets:       height x width
#    power_pellets: height x width
#    pac:           (row, col)
#    r:             (row, col)
#    b:             (row, col)
#    o:             (row, col)
#    p:             (row, col)
#    rf:            bool
#    bf:            bool
#    of:            bool
#    pf:            bool
#    dt:            distance threshold (in cells)




class GameEngine(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        # self.loop.add_reader(sys.stdin, self.keypress)
        # self.getActionInput = {
        #     "pellets": [()],
        #     "power_pellets": [()],
        #     "pac": (0,0),
        #     "r": 0,
        #     "b": 0,
        #     "o": 0,
        #     "p": 0,
        #     "rf": 0,
        #     "bf": 0,
        #     "of": 0,
        #     "pf": 0,
        #     "dt": 0,
        # }

        self.gameEngineGetAction = 0

    def update_state(state):
        return

    def get_action(state):
        return

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.LIGHT_STATE:


            # self.getActionInput = {
            #     "pellets": [()],
            #     "power_pellets": [()],
            #     "pac": (0,0),
            #     "r": 0,
            #     "b": 0,
            #     "o": 0,
            #     "p": 0,
            #     "rf": 0,
            #     "bf": 0,
            #     "of": 0,
            #     "pf": 0,
            #     "dt": 0, # frighten timer to cells
            # }

            # self.gameEngine.update(self.getActionInput)

            return
            # self.game.pacbot.update((msg.x, msg.y))

    def tick(self):
        # this function will get called in a loop with FREQUENCY frequency
        # if self.game.play:

        #     # call game engine
        #     self.gameEngine.getAction()

        #     # This will become asynchronous
        #     # self.game.next_step()
        # self._write_state()
        pacCommand = PacCommand()
        # pacCommand.mode = MsgType.PAC_COMMAND
        # command 1
        pacCommand.command_1.direction = 1
        pacCommand.command_1.forwards_distance = 1
        # command 2
        pacCommand.command_2.direction = 1
        pacCommand.command_2.forwards_distance = 2
        # command 3
        pacCommand.command_3.direction = 1
        pacCommand.command_3.forwards_distance = 3
        # command 4
        pacCommand.command_4.direction = 1
        pacCommand.command_4.forwards_distance = 4
        # command 5
        pacCommand.command_5.direction = 1
        pacCommand.command_5.forwards_distance = 5

        self.write(pacCommand, MsgType.PAC_COMMAND)
        print("sent pacCommand")
        return

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



