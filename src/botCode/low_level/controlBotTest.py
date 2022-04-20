import json
import sys
 
from motorsTesting import *

# goes forwards and stops at a wall
def mop(bot):
    # bot.move_forwards()
    bot.stop()
    while True:
        bot.arduino.read()
        print(str(bot.arduino.getOdometer()))
        if bot.arduino.readSensor("FORWARD_DIST") < 50:
            bot.arduino.resetOdometer()
            print("reset odometer")
    
        
        # char = input()
        # if char == "f":
        #     bot.move_forwards()
        #     print("forwards")
        # elif char == "b":
        #     print("backwards")
        #     bot.move_backwards()
        # elif char == "l":
        #     print("left")
        #     bot.rotate_left()
        # elif char == "r":
        #     print("right")
        #     bot.rotate_right()
        # elif char == "s":
        #     print("stop")
        #     bot.stop()
        # elif char == "a": # reset odometer
        #     print("reset odometer")
        #     bot.arduino.resetOdometer()

def main():
    bot = ArduinoMotors()
    try:
        mop(bot)
    except KeyboardInterrupt:
        bot.stop()

main()