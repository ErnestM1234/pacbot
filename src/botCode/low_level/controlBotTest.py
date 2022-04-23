import json
import sys
 
from sensorTesting import *

# goes forwards and stops at a wall
def mop(bot):

    while True:

        bot.write(FORWARDS, 1, False, False)

        if (bot.checkAck()):
            print("RECEIVED ACKNOWLEDGEMENT")

        # bot.write(FORWARDS, 0, True, False)

        # if (bot.checkAck()):
        #     print("RECEIVED ACKNOWLEDGEMENT")

        """command {
            direction: FORWARDS
            forwards_distance: 0
        }"""


def main():
    bot = ArduinoComms()
    try:
        mop(bot)
    except KeyboardInterrupt:
        bot.stop()

main()