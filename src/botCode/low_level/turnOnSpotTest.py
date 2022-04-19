import json

 
from motorsTesting import *

# goes forwards and stops at a wall
def main():
    bot = ArduinoMotors()

    bot.rotate_left()
    while True:
        bot.arduino.read()
        bot.rotate_left()

main()