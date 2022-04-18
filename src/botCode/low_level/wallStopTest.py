import json
from motorsTesting import *

# goes forwards and stops at a wall
def main():
    rightMotorDirection = 54
    rightMotorPower = 333
    leftMotorDirection = 0
    leftMotorPower = 4

    rmd = str(rightMotorDirection).zfill(3)
    rmp = str(rightMotorPower).zfill(3)
    lmd = str(leftMotorDirection).zfill(3)
    lmp = str(leftMotorPower).zfill(3)

    output = "{rmd:" + rmd + ",rmp:" + rmp + ",lmd:" + lmd + ",lmp:" + lmp + "}"
    print (output)
    # bot = ArduinoMotors()

    # bot.moveForwards(1)
    # while True:
    #     bot.arduino.read()
    #     if bot.arduino.readSensor("FORWARD_DIST") < WALL_STOP_DIST:
    #         bot.stop()
    #         print("done")
    #         return

main()