from enum import Enum
import serial
import json
import numpy as np

"""
SENSOR INPUTS:

LEFT_ENCODER    - float (cm)
RIGHT_ENCODER   - float (cm)

LEFT_DISTANCE   - float (cm)
RIGHT_DISTANCE  - float (cm)

LEFT_DIAG_DIST  - float (cm)
RIGHT_DIAG_DIST - float (cm)

FORWARD_DIST    - float (cm)

HEADING         - float (degree)

input json string format: 

'{ 
    "LEFT_ENCODER":0.0, "RIGHT_ENCODER":0.0, 
    "LEFT_DISTANCE":0.0, "RIGHT_DISTANCE":0.0,
    "LEFT_DIAG_DIST":0.0, "RIGHT_DIAG_DIST":0.0,
    "FORWARD_DIST":0.0, "HEADING":0.0
    }'


MOTOR OUTPUTS:

RIGHT_MOTOR_DIRECTION       - MotorDirection
RIGHT_MOTOR_POWER           - float (wats?? volts?? idk)

LEFT_MOTOR_DIRECTION       - MotorDirection
LEFT_MOTOR_POWER           - float (wats?? volts?? idk)

output json string format: '{
    "RIGHT_MOTOR_DIRECTION":0.0,
    "RIGHT_MOTOR_POWER":0.0,
    "LEFT_MOTOR_DIRECTION":0.0,
    "LEFT_MOTOR_POWER":0.0,
}'


"""

SENSOR_NAMES = {
    "ACCEL_X",  "ACCEL_Y",  "ACCEL_Z",
    "GYRO_X",   "GYRO_Y",   "GYRO_Z",
    "MAG_X",    "MAG_Y",    "MAG_Z"
}


class MotorDirection(Enum):
    BACKWARD = 0
    FORWARD = 1
    STOP = 2
    # todo: remove maintain by setting last power sent state and resending it again
    MAINTAIN = 3 # maintain current power 

class ArduinoComms:
    def __init__(self):
        self.sensors = {
            "LEFT_ENCODER":0.0,     "RIGHT_ENCODER":0.0, 
            "LEFT_DISTANCE":0.0,    "RIGHT_DISTANCE":0.0,
            "LEFT_DIAG_DIST":0.0,   "RIGHT_DIAG_DIST":0.0,
            "FORWARD_DIST":0.0,     "HEADING":0.0
        }
        self.ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        self.ser.reset_input_buffer()

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
        if sensor not in SENSOR_NAMES:
            return None
        return self.sensors[sensor]

    """
    Brian and Boaz
    This function returns the gyro reading of the pacbot as array
    """
    def getAccel(self):
        return np.array([self.readSensor("ACCEL_X"), self.readSensor("ACCEL_Y"),  self.readSensor("ACCEL_Z")])

    """
    Brian and Boaz
    This function returns the gyro reading of the pacbot as array
    """
    def getGyro(self):
        return np.array([self.readSensor("GYRO_X"), self.readSensor("GYRO_Y"),  self.readSensor("GYRO_Z")])

    """
    Brian and Boaz
    This function returns the magnetometer reading of the pacbot as array
    """
    def getMag(self):
        return np.array([self.readSensor("MAG_X"), self.readSensor("MAG_Y"),  self.readSensor("MAG_Z")])


    """
    Brian and Boaz
    This function returns the heading of the pacbot (current direction it is facing).
    This function returns an integer number from 0 to 360.
    North, on the game field, is 0 degrees.
    """
    def getHeading(self):

        ACCEL_X = self.getAccel()[0]
        ACCEL_Y = self.getAccel()[1]
        ACCEL_Z = self.getAccel()[2]

        ACCEL = np.array([ACCEL_X, ACCEL_Y, ACCEL_Z])

        """
        GYRO_X = self.getGyro()[0]
        GYRO_Y = self.getGyro()[1]
        GYRO_Z = self.getGyro()[2]
        """


        MAX_X = self.getMag()[0]
        MAG_Y = self.getMag()[1]
        MAG_Z = self.getMag()[2]

        # Algorithm for converting gyro and mag to heading
        # https://stackoverflow.com/questions/35061294/how-to-calculate-heading-using-gyro-and-magnetometer

        # Calculte aux
        unit_x = np.array([1, 0, 0])
        aux = np.cross(unit_x, np.cross(ACCEL, unit_x))

        # Calculate roll
        aux_y = aux[1]
        aux_z = aux[2]
        roll = np.arctan(aux.)

        # Calculate pitch

        # Calculate heading

        return 


    """
    Reads a new set of values from the serial stream.
    """
    def read(self):
        if self.ser.in_waiting > 0:
            sensor_input = self.ser.readline().decode('utf-8').rstrip()
            try:
                sensor_input_json = json.loads(sensor_input)
                self.sensors = sensor_input_json.items()
                # for sensor_name, value in sensor_input_json.items():
                #   self.sensors[sensor_name] = value
            except:
                print("input not in valid JSON format or maybe something else went wrong idk")

        return self.sensors

    """
    Input:
        r_dir   - direction of right motor spin
        r_pwr   - power of right motor
        l_dir   - direction of left motor spin
        l_pwr   - power of left motor
    Writes to command of format SENSOR_NAMES
    """
    def write(self, r_dir, r_pwr, l_dir, l_pwr):
        # todo: check that values are valid
        output = {
            "RIGHT_MOTOR_DIRECTION":r_dir,
            "RIGHT_MOTOR_POWER":r_pwr,
            "LEFT_MOTOR_DIRECTION":l_dir,
            "LEFT_MOTOR_POWER":l_pwr,
        }
        self.ser.write(json.dumps(output).encode('utf-8'))

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

