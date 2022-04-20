from audioop import avg
from enum import Enum
import math
import serial
import json
import numpy as np
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import sqrt as sqrt
from math import pi as pi
import time


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

SENSOR_NAMES = {
    "ACCEL_X",  "ACCEL_Y",  "ACCEL_Z",
    "GYRO_X",   "GYRO_Y",   "GYRO_Z",
    "MAG_X",    "MAG_Y",    "MAG_Z"
}
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


"""
ArduinoComms methods:
closeComms()
readSensor()
getHeading() ** unimplemented **
getOdometer() ** unimplemented **
resetOdometer()
read()
write()
print_all_values()
"""

class ArduinoComms:
    def __init__(self):
        self.sensors = {"LEFT_ENCODER":0,     "RIGHT_ENCODER":0, 
    "LEFT_DISTANCE":0,    "RIGHT_DISTANCE":0,
    "LEFT_DIAG_DIST":0,   "RIGHT_DIAG_DIST":0,  "FORWARD_DIST":100,
    "ACC_X":0,            "ACC_Y":0,            "ACC_Z":0,
    "GYRO_X":0,           "GYRO_Y":0,           "GYRO_Z":0,
    "MAG_X":0,            "MAG_Y":0,            "MAG_Z":0} # holds the most recent signal from arduino to pi
        self.motorState = {"rmd":0,"rmp":0,"lmd":0,"lmp":0} # holds most recent signal sent from pi to arduino
        # open connection to arduino
        self.ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.ser.readline()

        self.heading = 90 # pacbot starts by facing east
        self.odometer = 0
        self.odometer_left = 0
        self.odometer_right = 0

        self.last_time_measured = time.time_ns()


    """ closeComms()
    Input:  void
    Output: void
    Closes serial ports. Call this at the end of using Ardunio Comms
    """
    def closeComms(self):
        self.ser.close()

    """ readSensor()
    Input:  sensor - a valid sensor name
    Output: the current value of the specified sensor (does not run update)
    """
    def readSensor(self, sensor):
        if sensor not in SENSOR_NAMES:
            return None
        return self.sensors[sensor]

    """ getAccel()
    Input:  void
    Output: array of accelerometer values
    Returns numpy array of accelerometer X, Y, Z values
    """
    def getAccel(self):
        return np.array([self.readSensor("ACCEL_X"), self.readSensor("ACCEL_Y"),  self.readSensor("ACCEL_Z")])

    """ getGyro()
    Input:  void
    Output: array of gyroscope values
    Returns numpy array of gyroscope X, Y, Z values
    """
    def getGyro(self):
        return np.array([self.readSensor("GYRO_X"), self.readSensor("GYRO_Y"),  self.readSensor("GYRO_Z")])

    """ getMag()
    Input:  void
    Output: array of magnetometer values
    Returns numpy array of magnetometer X, Y, Z values
    """
    def getMag(self):
        return np.array([self.readSensor("MAG_X"), self.readSensor("MAG_Y"),  self.readSensor("MAG_Z")])


    """ getHeading()
    Input: void
    Output: heading 0 to 360
    Returns the heading of the pacbot (current direction it is facing), an integer number from 0 to 360.
    North, on the game field, is 0 degrees.
    """
    def getHeading(self):

        ACCEL_X = self.getAccel()[0]
        ACCEL_Y = self.getAccel()[1]
        ACCEL_Z = self.getAccel()[2]
        
        ACCEL = np.array([ACCEL_X, ACCEL_Y, ACCEL_Z])

        MAG_X = self.getMag()[0]
        MAG_Y = self.getMag()[1]
        MAG_Z = self.getMag()[2]
        
        # Calculte aux
        unit_x = np.array([1, 0, 0])
        aux = np.cross(unit_x, np.cross(ACCEL, unit_x))

        # Calculate roll
        aux_y = aux[1]
        aux_z = aux[2]
        roll = np.arctan2(aux_y/aux_z)

        # Calculate pitch
        y = -ACCEL_X
        x = sqrt(ACCEL_Y**2 + ACCEL_Z**2)
        pitch = atan2(y, x)

        # Calculate heading
        y = -MAG_Y*cos(roll) + MAG_Z*sin(roll)
        x = MAG_X*cos(pitch) + MAG_Y*sin(roll)*sin(pitch) + MAG_Z*cos(roll)*sin(pitch)
        heading = atan2(y, x) * 180/pi

        return heading 

    """ getOdometer()
    input:  void
    output: odometer value (mm)
    Returns the current value of the odometer
    """
    def getOdometer(self):
        # todo: this function
        # I think encoders return the position of the motor
        # we can measure the average value for how much each individual encoder moves
        # self.odometer = ((self.odometer_left - self.sensors["LEFT_ENCODER"]) + (self.odometer_right - self.sensors["RIGHT_ENCODER"])) / 2
        return self.odometer
    
    """ resetOdometer()
    input:  void
    output: void
    sets odometer value to 0
    """
    def resetOdometer(self):
        # self.odometer_left = self.sensors["LEFT_ENCODER"]
        # self.odometer_right = self.sensors["RIGHT_ENCODER"]
        self.odometer = 0

    """ resetOdometer()
    input:  void
    output: void
    update odometer
    """
    def updateOdometer(self):
        curr_time = time.time_ns()
        d_time = (curr_time - self.last_time_measured)
        avg_enc = ((self.sensors["LEFT_ENCODER"]) + (self.sensors["RIGHT_ENCODER"])) / 2
        rpm_to_radians_per_sec = math.pi * 2 / 60000000000 
        self.odometer += avg_enc * d_time * rpm_to_radians_per_sec
        self.last_time_measured = curr_time

    """ read()
    input:  void
    output: a python dictionary containing all of the sensor values
    Reads a new set of values from the serial stream.
    """
    def read(self):
        if self.ser.in_waiting > 0:
            sensor_input = self.ser.readline().decode('ascii').rstrip()
            # this is to ensure that we are receiving a json formatted string
            # print(sensor_input)
            for key, value in json.loads(sensor_input).items():
                self.sensors[key] = value
        
            # update odometer
            # print("L enc: " + str(self.sensors["LEFT_ENCODER"]))
            # print("R enc: " + str(self.sensors["RIGHT_ENCODER"]))
            self.updateOdometer()
            print("odometer:" + str(self.getOdometer()))

        return self.sensors

    """ write()
    Input:
        rightMotorDirection     - direction of right motor spin
        rightMotorPower         - power of right motor
        leftMotorDirection      - direction of left motor spin
        leftMotorPower          - power of left motor
    Output:     void
    Writes to command of format {rmd:000,rmp:000,lmd:000,rmp:000}
    """
    def write(self, rightMotorDirection, rightMotorPower, leftMotorDirection, leftMotorPower):
        # check input validity
        rmpVerified = max(min(rightMotorPower,255),0) # 0 < rmd < 255
        lmpVerified = max(min(leftMotorPower, 255),0) # 0 < lmd < 255


        # output integer format must be in three digits
        self.motorState["rmd"] = rightMotorDirection
        self.motorState["rmp"] = rmpVerified
        self.motorState["lmd"] = leftMotorDirection
        self.motorState["lmp"] = lmpVerified


        # convert to string
        # rmd = str(rmdVerified).zfill(3)
        rmp = str(rmpVerified).zfill(3)
        # lmd = str(lmdVerified).zfill(3)
        lmp = str(lmpVerified).zfill(3)
        # output = "{rmd:" + rmd + ",rmp:" + rmp + ",lmd:" + lmd + ",lmp:" + lmp + "}"

        if rightMotorDirection == MotorDirection.FORWARDS:
            rmd = "000"
        else:
            rmd = "001"

        if leftMotorDirection == MotorDirection.FORWARDS:
            lmd = "000"
        else:
            lmd = "001"
        
        output = "{rmd:" + rmd + ",rmp:" + rmp + ",lmd:" + lmd + ",lmp:" + lmp + "}"

        print(output)

        # for testing purposes comment out when not testing
        # self.simulation_update(rmdVerified, rmpVerified, lmdVerified, lmpVerified)

        # write to serial
        self.ser.write(output.encode('utf-8'))

    """ print_all_values()
    input:  void
    output: void
    prints the sensor values in a dictionary
    """
    def print_all_values(self):
        print(self.sensors)


    # ------------------------ Simulation ------------------------ #
    def simulation_update(self, rmd, rmp, lmd, lmp):
        # update odometer
        if rmd == MotorDirection.FORWARDS and lmd == MotorDirection.FORWARDS:
            self.odometer += (rmp + lmp) / 2
        elif rmd == MotorDirection.BACKWARDS and lmd == MotorDirection.BACKWARDS:
            self.odometer -= (rmp + lmp) / 2
        
        # update heading
        elif rmd == MotorDirection.FORWARDS and lmd == MotorDirection.BACKWARDS: # right turn
            print("----- right turn -----")
            self.heading += 10
        elif rmd == MotorDirection.BACKWARDS and lmd == MotorDirection.FORWARDS: # left turn
            print("----- left turn -----")
            self.heading -= 10
    
    def simulation_reset_odometer(self):
        self.odometer = 0

    def simulation_get_odometer(self):
        return self.odometer

    def simulation_print_state(self):
        print("heading: " + str(self.heading) + ", odometer: " + str(self.odometer))



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

