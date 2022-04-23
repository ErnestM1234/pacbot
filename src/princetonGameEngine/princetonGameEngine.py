import os, sys, logging
import time
import robomodules as rm
from messages import *
from pacbot.variables import game_frequency, ticks_per_update
from pacbot import StateConverter, GameState
from messages.pacCommand_pb2 import PacCommand



# for high level
import numpy as np

FREQUENCY = game_frequency * ticks_per_update


# ADDRESS = os.environ.get("BIND_ADDRESS","localhost") # the address of the game engine server
# ADDRESS = os.environ.get("BIND_ADDRESS","localhost")
ADDRESS = os.environ.get("BIND_ADDRESS","172.20.10.3")
PORT = os.environ.get("BIND_PORT", 11293)            # the port the game engine server is listening to


# high level code
I = 1
o = 2
e = 3
O = 4
n = 5
c = 6
grid = [[I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I], # 0
        [I,o,o,o,o,I,I,O,o,o,o,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I], # 5
        [I,o,I,I,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I], # 10
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,o,o,o,I,I,o,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,e,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,I,I,I,I,I,e,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,o,o,o,I,I,o,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I], # 15
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I], # 20
        [I,o,I,I,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I], # 25
        [I,o,o,o,o,I,I,O,o,o,o,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I]]
#        |         |         |         |         |         |         |   top right of pacman board
#        0         5        10        15       20         25       30
# o = normal pellet, e = empty space, O = power pellet, c = cherry position
# I = wall, n = ghost chambers
WALLS = np.logical_or(np.array(grid) == I, np.array(grid) == n, dtype=bool)
ACTIONS = [(-1, 0), (0, -1), (0, 0), (0, 1), (1, 0)]
NT = 3

class Node():
        """A node class for A* Pathfinding"""

        def __init__(self, parent=None, position=None):
            self.parent = parent
            self.position = position

            self.g = 0
            self.h = 0
            self.f = 0

        def __eq__(self, other):
            return self.position == other.position

def astar(maze, start, prev_start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    start = tuple(start)
    end = tuple(end)

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            path = path[::-1]
            if len(path) < 3:
                    return [(0, 1)]
            new_path = []
            distance = 0
            for i in range(len(path) - 2):
                change = np.subtract(path[i + 1], path[i])
                diff = np.subtract(change, np.subtract(path[i + 2], path[i + 1]))
                if diff[0] and diff[1]:
                    new_path.append((0, distance + 1))
                    distance = 0
                    if change[0]:
                        new_path.append((diff[1], -1))
                    else:
                        new_path.append((diff[0], -1))
                else:
                    distance += 1
            if distance != 0:
                new_path.append((0, distance))
            return new_path # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares
            if (current_node.position == start):
                back = np.subtract(start, prev_start) * -1
                if new_position == back:
                    continue
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            skip = False
            for closed_child in closed_list:
                if child == closed_child:
                    skip = True
                    break
            if skip:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            on = False
            for open_node in open_list:
                if child == open_node:
                    on = True
                    if child.g > open_node.g:
                        skip = True
            if skip:
                continue

            if on:
                open_list.remove(child)

            # Add the child to the open list
            open_list.append(child)


def runAStar():

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = astar(maze, start, end)
    print(path)


# helper method to astar to a ghost, which is technically a barrier in maze
def astar_ghost(maze, start, prev_start, end):
    maze[end] = False
    path = astar(maze, start, prev_start, end)
    maze[end] = True
    return path

# state is a dict with keys:
#    prev_pac:      (row, col)
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
def get_action(state):
    # first coordinate 0, 1, -1 depending on forward, left, right
    obstacles = WALLS.copy()
    g_positions = []
    f_positions = []
    # consider ghosts which are not frightened to be obstacles
    if state["rf"]:
        f_positions.append(state["r"])
    else:
        g_positions.append(state["r"])
        obstacles[state["r"]] = True
    if state["bf"]:
        f_positions.append(state["b"])
    else:
        g_positions.append(state["b"])
        obstacles[state["b"]] = True
    if state["of"]:
        f_positions.append(state["o"])
    else:
        g_positions.append(state["o"])
        obstacles[state["o"]] = True
    if state["pf"]:
        f_positions.append(state["p"])
    else:
        g_positions.append(state["p"])
        obstacles[state["p"]] = True
    # pass the first five things 
    # prevents pacbot from going backwards
    obstacles[state["prev_pac"]] = True
    
    print("phase: frightened ghosts")

    # target the closest frightened ghost not on pac
    # move to it if it exists and is within dt
    closest_d = None
    closest_path = None
    for f_position in f_positions:
        print("pathfinding to frightened ghost")
        path = astar_ghost(obstacles, state["pac"], f_position)
        if not path or len(path) < 2:
            continue
        if closest_d is None or closest_d > len(path) - 1:
            closest_d = len(path) - 1
            closest_path = path
            if closest_d <= 1:
                break
    if closest_d and closest_d <= state["dt"]:
        return path

    print("phase: power pellets")

    # target the closest power pellet not on pac
    # move to it, if it exists and (is further than 1 cell away or a ghost is within NT)
    # wait at it, if it exists and is within 1 cell and a ghost is not within NT cells
    nearby = False
    for g_position in g_positions:
        if WALLS[g_position]:
            continue
        print("pathfinding to ghost")
        path = astar_ghost(obstacles, state["pac"], g_position)
        if not path or len(path) - 1 <= NT:
            nearby = True
            
    print("nearby:", nearby)
    positions = np.argwhere(state["power_pellets"])
    closest_d = None 
    closest_path = None
    for position in positions:
        print("pathfinding to power pellet")
        path = astar(obstacles, state["pac"], state["prev_pac"], position)
        if not path or len(path) < 2:
            continue 
        if closest_d is None or closest_d > len(path) - 1:
            closest_d = len(path) - 1
            closest_path = path 
            if closest_d <= 1:
                break
    if closest_d:
        if closest_d > 1 or nearby:
            return closest_path
        else:
            return [(0, 0)]
    # grid, algorithm, a star
    print("phase: pellets")
    # target the closest pellet not on pac
    # move to it if it exists
    positions = np.argwhere(state["pellets"])
    

    closest_d = None 
    closest_path = None
    for position in positions:
        path = astar(obstacles, state["pac"], state["prev"], position)
        if not path or len(path) < 2:
            continue 
        if closest_d is None or closest_d > len(path) - 1:
            closest_d = len(path) - 1
            closest_path = path 
            if closest_d <= 1:
                break
    if closest_d:
        return closest_path

    return [(0, 0)]    



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


class GameEngine(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, int(FREQUENCY), self.subscriptions)
        # self.loop.add_reader(sys.stdin, self.keypress)
        self.game = GameState()
        self.state = {
            "prev_pac": (0,0), # spot behind pacman
            "pellets": [(0,0)],
            "power_pellets": [(0,0)],
            "pac": (0,0),
            "r": (0,0),
            "b": (0,0),
            "o": (0,0),
            "p": (0,0),
            "rf": False,
            "bf": False,
            "of": False,
            "pf": False,
            "dt": 0, # frighten timer to cells
        }

        self.gameEngineGetAction = 0


    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.LIGHT_STATE:
            
            rgf = False # red ghost frightened
            try:
                rgf = msg.red_ghost.state
            except: 
                print("no redghost")

            bgf = False
            try:
                bgf = msg.blue_ghost.state
            except: 
                print("no redghost")

            pgf = False
            try:
                pgf = msg.pink_ghost.state
            except: 
                print("no redghost")

            ogf = False
            try:
                ogf = msg.orange_ghost.state
            except: 
                print("no redghost")
            

            self.getActionInput = {
                "prev_pac": (3,3), # spot behind pacman
                "pellets": [(0,0)],
                "power_pellets": [(0,0)],
                "pac": (msg.pacman.x, msg.pacman.y),
                "r": (msg.red_ghost.x, msg.red_ghost.y),
                "b": (msg.blue_ghost.x, msg.blue_ghost.y),
                "o": (msg.orange_ghost.x, msg.orange_ghost.y),
                "p": (msg.blue_ghost.x, msg.blue_ghost.y),
                "rf": rgf,
                "bf": bgf,
                "of": ogf,
                "pf": pgf,
                "dt": 0, # frighten timer to cells
            }

            self.state = self.getActionInput



    def tick(self):
        # this function will get called in a loop with FREQUENCY frequency
        # if self.game.play:

        #     # call game engine
        #     self.gameEngine.getAction()

        #     # This will become asynchronous
        #     # self.game.next_step()
        # self._write_state()

        # index 0 is dir (int)
        # index 1 is forwards dist (int)

        
        pacActionCommand = get_action(self.state)[0] # get command from algorithm


        # fill out pac command
        pacCommand = PacCommand()

        direction = PacCommand.FORWARDS
        forwards_distance = 0

        if pacActionCommand[0] == 0:
            direction = PacCommand.FORWARDS
            forwards_distance = pacActionCommand[1]
        elif pacActionCommand[0] == 1:
            direction = PacCommand.LEFT
        elif pacActionCommand[0] == -1:
            direction = PacCommand.RIGHT

            
        pacCommand.command.direction = direction
        pacCommand.command.forwards_distance = forwards_distance
        self.write(pacCommand.SerializeToString(), MsgType.PAC_COMMAND)

        
        # pacCommand = PacCommand()
        # pacCommand.command.direction = PacCommand.FORWARDS
        # pacCommand.command.forwards_distance = 1
        # self.write(pacCommand.SerializeToString(), MsgType.PAC_COMMAND)
        # print(str(pacCommand))

        # light_state = StateConverter.convert_game_state_to_light(self.game)
        # self.write(light_state.SerializeToString(), MsgType.LIGHT_STATE)
        # print("sent mess")

        # pacCommand = PacCo mmand()
        # direction = PacCommand.FORWARDS
        # forwards_distance = 1
        # pacCommand.command.direction = direction
        # pacCommand.command.forwards_distance = forwards_distance


        

        # light_state = StateConverter.convert_game_state_to_light(self.game)
        # self.write(light_state.SerializeToString(), MsgType.LIGHT_STATE)

        # self.write(light_state, MsgType.PACMAN_COMMAND)

        # print("\n ----- sent pacCommand: \n" + str(pacCommand) + "----- \n")
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



