
from motorsTesting import *

# goes forwards and stops at a wall
def main():
    bot = ArduinoMotors()

    bot.goForwards(1)
    while True:
        bot.arduino.read()
        if bot.arduino.readSensor("FORWARD_DIST") < WALL_STOP_DIST:
            bot.stop()
            print("done")
            return

main()