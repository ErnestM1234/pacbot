#High Level Game code for PacGhosts
from variables import *
from g import *
from classes import *
from display import *
import layout
from graphicsDisplay import *
from BotTracker import *
from makePacbot import *
import random

from threading import Thread, Lock
import time
lock = Lock()

lay = layout.getLayout("pacbotLayout.lay")
pacbot = PacBot((23,26), "right")

game = GameState(grid, lay, pacbot)
botTracker = BotTracker()
graphics = None

def main():
    global lay, pacbot, game, botTracker
    graphics = PacmanGraphics(0.5)

    graphics.initialize(game.gstate)

    
    position = (23,26)
    direction = botTracker.get_bot_direction()
    pacbot.update(position, direction)

    Thread(target = trackerUpdate).start()
    Thread(target = gameUpdate(graphics)).start()

def trackerUpdate():

    while game.game_on: 
        lock.acquire()
        position = (-1,-1)
        direction = botTracker.get_bot_direction()
        pacbot.update(position, direction)
        try:
            
            game.game_go(pacbot)
            # if game.grid[position[0]][position[1]] != I and game.grid[position[0]][position[1]] != n:
            #     print("true")
            # else:
            #     print("false")
            # print(str(game.pacbot.pos[0]) + " " + str(game.pacbot.pos[1]))
        finally:
            lock.release()
        time.sleep(.1)


def gameUpdate(graphics):       
    # display start postions
    display_game(game.pacbot,game.red,game.pink,game.orange,game.blue,game.score,game.lives,game.state,game.grid)


    if game.grid[game.pacbot.pos[0]][game.pacbot.pos[1]] == o:
        game.grid[game.pacbot.pos[0]][game.pacbot.pos[1]] = e
        game.score += 1
    
    counter = 1
    while game.game_on:
    
        # if counter < 2:
        #     graphics.update(game.gstate)
        #     counter += 1
        # else:
        #     counter = 1
        position =(23,12)
        direction = botTracker.get_bot_direction()

        graphics.update(game.gstate)
        pacbot.update(position, direction)

        lock.acquire()
        try:
           
            game.game_go(pacbot, False)

        finally:
            lock.release()




                        
if __name__ == "__main__":
    main()




