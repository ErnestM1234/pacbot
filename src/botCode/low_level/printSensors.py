import json
import time
from motorsTesting import *
from sensorTesting import *

# goes forwards and stops at a wall
def main():
    comms = ArduinoComms()

    while True:
        m = comms.read()
        print(str(m))
        time.sleep(4)

main()