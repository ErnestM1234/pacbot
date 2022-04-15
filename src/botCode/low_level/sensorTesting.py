from enum import Enum
import serial
import json

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

input json string format: '{ 
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
    "LEFT_ENCODER",     "RIGHT_ENCODER", 
    "LEFT_DISTANCE",    "RIGHT_DISTANCE",
    "LEFT_DIAG_DIST",   "RIGHT_DIAG_DIST",
    "FORWARD_DIST",     "HEADING"
}


class MotorDirection(Enum):
    BACKWARD = 0
    FORWARD = 1
    STOP = 2
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


    def readSensor(self, sensor):
        if sensor not in SENSOR_NAMES:
            return None
        return self.sensors[sensor]

    # update sensors
    def read(self):
        if self.ser.in_waiting > 0:
            sensor_input = self.ser.readline().decode('utf-8').rstrip()
            try:
                sensor_input_json = json.loads(sensor_input)
                for sensor_name, value in sensor_input_json.items():
                    self.sensors[sensor_name] = value
            except:
                print("input not in valid JSON format or maybe something else went wrong idk")

        return self.sensors

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

