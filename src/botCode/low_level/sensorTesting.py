from enum import Enum
import serial
import json

"""
INPUTS:

LEFT_ENCODER    - float (mm)
RIGHT_ENCODER   - float (mm)

LEFT_DISTANCE   - float (mm)
RIGHT_DISTANCE  - float (mm)
LEFT_DIAG_DIST  - float (mm)
RIGHT_DIAG_DIST - float (mm)
FORWARD_DIST    - float (mm)

ACC_X           - integer (??)
ACC_Y           - integer (??)
ACC_Z           - integer (??)

GYRO_X          - integer (??)
GYRO_Y          - integer (??)
GYRO_Z          - integer (??)

MAG_X           - integer (??)
MAG_Y           - integer (??)
MAG_Z           - integer (??)


input json string format (no new lines until end): 

{ 
"LEFT_ENCODER":0,     "RIGHT_ENCODER":0, 
"LEFT_DISTANCE":0,    "RIGHT_DISTANCE":0,
"LEFT_DIAG_DIST":0,   "RIGHT_DIAG_DIST":0,  "FORWARD_DIST":0,
"ACC_X":0,            "ACC_Y":0,            "ACC_Z":0,
"GYRO_X":0,           "GYRO_Y":0,           "GYRO_Z":0,
"MAG_X":0,            "MAG_Y":0,            "MAG_Z":0
}\n


OUTPUTS:

rmd         - right motor direction     int (FORWARDS = 0, BACKWARDS = 1, STOP = 2)
rmp         - right motor power         int (0-255)
lmd         - left motor direction      int (FORWARDS = 0, BACKWARDS = 1, STOP = 2)
lmp         - left motor power          int (0-255)

output json string format:
{rmd:000,rmp:000,lmd:000,rmp:000}

"""

SENSOR_NAMES = [
    "LEFT_ENCODER",     "RIGHT_ENCODER", 
    "LEFT_DISTANCE",    "RIGHT_DISTANCE",
    "LEFT_DIAG_DIST",   "RIGHT_DIAG_DIST",  "FORWARD_DIST",
    "ACC_X",            "ACC_Y",            "ACC_Z",
    "GYRO_X",           "GYRO_Y",           "GYRO_Z",
    "MAG_X",            "MAG_Y",            "MAG_Z"
]


class MotorDirection(Enum):
    FORWARDS = 0
    BACKWARDS = 1
    STOP = 2

class ArduinoComms:
    def __init__(self):
        self.sensors = {} # holds the most recent signal from arduino to pi
        self.motorState = {"rmd":0,"rmp":0,"lmd":0,"lmp":0} # holds most recent signal sent from pi to arduino
        self.ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.heading = 0

    """
    Closes serial ports. Call this at the end of using Ardunio Comms
    """
    def closeComms(self):
        self.ser.close()

    """
    Input:
        sensor - a valid sensor name
    Returns:
        the current value of the specified sensor (does not run update)
    """
    def readSensor(self, sensor):
        try:
            sensorValue = self.sensors[sensor]
        except:
            sensorValue = 0
        
        return sensorValue

    """
    Brian
    This function returns the heading of the pacbot (current direction it is facing).
    This function returns an integer number from 0 to 360.
    North, on the game field, is 0 degrees.
    """
    def getHeading(self):
        return 0

    """
    Reads a new set of values from the serial stream.
    """
    def read(self):
        if self.ser.in_waiting > 0:
            sensor_input = self.ser.readline().decode('utf-8').rstrip()
            # this is to ensure that we are receiving a json formatted string
            try:
                self.sensors = json.loads(sensor_input)
            except:
                print("failed to parse json")

        return self.sensors

    """
    Input:
        rightMotorDirection     - direction of right motor spin
        rightMotorPower         - power of right motor
        leftMotorDirection      - direction of left motor spin
        leftMotorPower          - power of left motor
    Writes to command of format {rmd:000,rmp:000,lmd:000,rmp:000}
    """
    def write(self, rightMotorDirection, rightMotorPower, leftMotorDirection, leftMotorPower):
        # check input validity
        rmdVerified = rightMotorDirection if (rightMotorDirection != 0 or rightMotorDirection != 1) else 2 # if not known value return stop
        lmdVerified = leftMotorDirection  if (leftMotorDirection  != 0 or leftMotorDirection  != 1) else 2 # if not known value return stop
        rmpVerified = max(min(rightMotorPower,255),0) # 0 < rmd < 255
        lmpVerified = max(min(leftMotorPower, 255),0) # 0 < lmd < 255


        # output integer format must be in three digits
        self.motorState["rmd"] = rmdVerified
        self.motorState["rmp"] = rmpVerified
        self.motorState["lmd"] = lmdVerified
        self.motorState["lmp"] = lmpVerified


        # convert to string
        rmd = str(rmdVerified).zfill(3)
        rmp = str(rmpVerified).zfill(3)
        lmd = str(lmdVerified).zfill(3)
        lmp = str(lmpVerified).zfill(3)
        output = "{rmd:" + rmd + ",rmp:" + rmp + ",lmd:" + lmd + ",lmp:" + lmp + "}"

        # write to serial
        self.ser.write(output.encode('utf-8'))

    def print_all_values(self):
        print(self.sensors)


# if __name__ == '__main__':
#     ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
#     ser.reset_input_buffer()
    
#     while True:
#         if ser.in_waiting > 0:
#             sensor_input = ser.readline().decode('utf-8').rstrip()
#             print(sensor_input)
            
#             try:
#                 sensor_input_json = json.dumps(sensor_input)
#             except:
#                 print("input not in valid JSON format")
            
#             # ser.write('Hello from PI\n'.encode('utf-8'))

