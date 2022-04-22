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

SENSOR_NAMES = [
    "LEFT_ENCODER",     "RIGHT_ENCODER", 
    "LEFT_DISTANCE",    "RIGHT_DISTANCE",
    "LEFT_DIAG_DIST",   "RIGHT_DIAG_DIST",  "FORWARD_DIST",
    "ACC_X",            "ACC_Y",            "ACC_Z",
    "GYRO_X",           "GYRO_Y",           "GYRO_Z",
    "MAG_X",            "MAG_Y",            "MAG_Z"
    "HEADING",
]

RPM_TO_RADIANS_PER_NANSEC = math.pi * 2 / 60000000000 

class MotorDirection(Enum):
    FORWARDS = 0
    BACKWARDS = 1
    STOP = 2


"""
ArduinoComms methods:
closeComms()
readSensor()
getHeading()
getOdometer()
resetOdometer()
read()
write()
print_all_values()
"""

class ArduinoComms:
    def __init__(self):
        self.sensors = {
            "LEFT_ENCODER":0, "RIGHT_ENCODER":0, "LEFT_DISTANCE":100, "RIGHT_DISTANCE":100,
            "LEFT_DIAG_DIST":100, "RIGHT_DIAG_DIST":100, "FORWARD_DIST":100,
            "ACC_X":0, "ACC_Y":0, "ACC_Z":0,
            "GYRO_X":0, "GYRO_Y":0, "GYRO_Z":0,
            "MAG_X":0, "MAG_Y":0, "MAG_Z":0,
            "HEADING":0
        } # holds the most recent signal from arduino to pi
        self.motorState = {"rmd":0,"rmp":0,"lmd":0,"lmp":0} # holds most recent signal sent from pi to arduino
        # open connection to arduino
        self.ser = serial.Serial('/dev/ttyS0', 38400, timeout=1)
        self.ser.reset_input_buffer()
        self.ser.readline()

        self.heading = 90 # pacbot starts by facing east
        self.odometer = 0
        self.odometer_left = 0
        self.odometer_right = 0

        self.last_time_measured = time.time_ns()
        self.last_gyro_time = time.time_ns()

        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

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
            print("sensor " + sensor + " not found")
            return None
        return self.sensors[sensor]

    """ getAccel()
    Input:  void
    Output: array of accelerometer values
    Returns numpy array of accelerometer X, Y, Z values
    """
    def getAccel(self):
        return np.array([self.readSensor("ACC_X"), self.readSensor("ACC_Y"),  self.readSensor("ACC_Z")])

    """ getGyro()
    Input:  void
    Output: array of gyroscope values
    Returns numpy array of gyroscope X, Y, Z values
    """
    def getGyro(self):

        d_time = time.time() - self.last_gyro_time
        GYRO_X = self.readSensor("GYRO_X") / 4375;
        GYRO_Y = self.readSensor("GYRO_Y") / 4375;
        GYRO_Z = self.readSensor("GYRO_Z") / 4375;

        # self.gyro_x += GYRO_X * d_time
        # self.gyro_y += GYRO_Y * d_time
        # self.gyro_z += GYRO_Z * d_time

        if abs(GYRO_X) > 1:
            self.gyro_x += GYRO_X * d_time
        if abs(GYRO_Y) > 1:
            self.gyro_y += GYRO_Y * d_time
        if abs(GYRO_Z) > 1:
            self.gyro_z += GYRO_Z * d_time

        # print("GYRO_X: " + "{:4.2f}".format(self.gyro_x) + " GYRO_Y: " + "{:4.2f}".format(self.gyro_y) + " GYRO_Z: " + "{:4.2f}".format(self.gyro_z))

        self.last_gyro_time = time.time()

        return np.array([GYRO_X, GYRO_Y, GYRO_Z])

    """ getMag()
    Input:  void
    Output: array of magnetometer values
    Returns numpy array of magnetometer X, Y, Z values
    """
    def getMag(self):
        # MAG_X = self.readSensor("MAG_X") / 2**15 * 6842
        # MAG_Y = self.readSensor("MAG_Y") / 2**15 * 6842
        # MAG_Z = self.readSensor("MAG_Z") / 2**15 * 6842
        MAG_X = self.readSensor("MAG_X") / 6842
        MAG_Y = self.readSensor("MAG_Y") / 6842
        MAG_Z = self.readSensor("MAG_Z") / 6842
        return np.array([MAG_X, MAG_Y, MAG_Z])


    """ getHeading()
    Input: void
    Output: heading 0 to 360
    Returns the heading of the pacbot (current direction it is facing), an integer number from 0 to 360.
    North, on the game field, is 0 degrees.
    """
    def getHeading(self):
        # print("self.sensors[HEADING]" + str(self.sensors["HEADING"]))
        return self.sensors["HEADING"] / 100

    """ getOdometer()
    input:  void
    output: odometer value (mm)
    Returns the current value of the odometer
    """
    def getOdometer(self):
        # todo: this function
        # I think encoders return the position of the motor
        # we can measure the average value for how much each individual encoder moves
        self.odometer = ((self.sensors["LEFT_ENCODER"]) + (self.sensors["RIGHT_ENCODER"])) / 2
        return self.odometer
    
    """ resetOdometer()
    input:  void
    output: void
    sets odometer value to 0
    """
    def resetOdometer(self):
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
        self.odometer += avg_enc * d_time * RPM_TO_RADIANS_PER_NANSEC
        self.last_time_measured = curr_time

    """ read()
    input:  void
    output: a python dictionary containing all of the sensor values
    Reads a new set of values from the serial stream.
    """
    def read(self):
        # read from input buffer
        if self.ser.in_waiting > 0:
            # flush input
            # self.ser.reset_input_buffer()
            # this is to ensure that we are receiving a json formatted string
            # try:
            sensor_input = self.ser.readline().decode('ascii').rstrip()

            # print("raw input " + sensor_input)
            # print("received input")
            
            if (len(sensor_input) > 0 and sensor_input[0] == '{' and sensor_input[len(sensor_input)-1] == '}'):
                temp_sensor_input = sensor_input.replace('{','')
                temp_sensor_input = temp_sensor_input.replace('}','')
                # print(temp_sensor_input)
                temp_sensor_data = temp_sensor_input.split(',')

                # for i in range(len(temp_sensor_data)):
                #     temp_sensor_data[i] = filter(str.isdigit, temp_sensor_data[i])
                
                # print(str(temp_sensor_data))
                
                for i in range(len(temp_sensor_data)-1):
                    # print(temp_sensor_data[i])
                    self.sensors[SENSOR_NAMES[i]] = int(float(temp_sensor_data[i]))
                
                # print(sensor_input)
                # print(str(self.sensors))
                # print("parsed correctly")
                print("self.sensors[]: " + str(self.sensors))
                self.ser.reset_input_buffer()



                # sensor_items = json.loads(sensor_input).items()
                # for key, value in sensor_items:
                #     self.sensors[key] = value
            # except Exception as e:
            #     print("parsing error: " + str(e))
                # print(sensor_input)
                # print(temp_sensor_data)
                # print("ACC_X: " + str(self.sensors["ACC_X"]) + " ACC_Y: " + str(self.sensors["ACC_Y"]) + " ACC_Z: " + str(self.sensors["ACC_Z"]))
                # print("GYRO_X: " + str(self.sensors["GYRO_X"]).zfill(8) + " GYRO_Y: " + str(self.sensors["GYRO_Y"]).zfill(8) + " GYRO_Z: " + str(self.sensors["GYRO_Z"]).zfill(8))
                # print("MAG_X: " + str(self.sensors["MAG_X"]) + " MAG_Y: " + str(self.sensors["MAG_Y"]) + " MAG_Z: " + str(self.sensors["MAG_Z"]))

            # self.ser.reset_input_buffer()
            # update odometer
            # self.updateOdometer()
            # print("odometer:" + str(self.getOdometer()))

            # d_time = (time.time_ns() - self.last_time_measured)
            # self.last_time_measured = time.time_ns()
            # print(str(d_time))
            


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

    """ calibrate()
    input: void
    output: void
    reset encoder values to 0
    """
    def calibrate(self):
        output = "{cal}"
        for i in range(5):
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

