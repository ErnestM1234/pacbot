import json
import sys
 
from motorsTesting import *

# goes forwards and stops at a wall
def mop(bot):
    # bot.move_forwards()
    bot.stop()
    while True:
        bot.arduino.read()
        # heading = bot.arduino.getHeading()
        # print("GYRO_X: " + "{:4.2f}".format(heading[0]) + " GYRO_Y: " + "{:4.2f}".format(heading[1]) + " GYRO_Z: " + "{:4.2f}".format(heading[2]))
        # print(str(bot.arduino.getOdometer()))
        # if bot.arduino.readSensor("FORWARD_DIST") < 50:
        #     bot.arduino.resetOdometer()
        #     print("reset odometer")
    
        #char = input()
        #if char == "n":
        print("MAG X: " + "{:6.2f}".format(bot.arduino.readSensor("MAG_X")) + " MAG Y: " + "{:6.2f}".format(bot.arduino.readSensor("MAG_Y")) + " MAG Z: " + "{:6.2f}".format(bot.arduino.readSensor("MAG_Z")))
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