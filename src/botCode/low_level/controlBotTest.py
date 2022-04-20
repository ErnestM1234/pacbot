import json
import sys
 
from motorsTesting import *

# goes forwards and stops at a wall
def main():
    
    bot = ArduinoMotors()
    bot.move_forwards()
    while True:
        bot.arduino.read()
        # print("heading:" + str(bot.arduino.getHeading()))
        # print(str(bot.arduino.getHeading()))
        # char = ""
        char = input()
        if char == "f":
            bot.move_forwards()
            print("forwards")
        elif char == "b":
            print("backwards")
            bot.move_backwards()
        elif char == "l":
            print("left")
            bot.rotate_left()
        elif char == "r":
            print("right")
            bot.rotate_right()
        elif char == "s":
            print("stop")
            bot.stop()
        elif char == "a": # reset odometer
            print("reset odometer")
            bot.arduino.resetOdometer()
main()