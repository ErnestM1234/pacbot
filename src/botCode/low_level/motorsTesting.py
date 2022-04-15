from sensorTesting import *

WALL_STOP_DIST = 5 # cm

class AvailableMotors(Enum):
    LEFT = 0
    RIGHT = 1


# this does not correspond directly to real motor, this is more of an abstract motor
class ArduinoMotor:
    def __init__(self, arduino, motor_state):
        self.motor_state = motor_state
        self.arduino = arduino

    def move(self, dir, pwr):
        if self.motor_state == AvailableMotors.RIGHT:
            self.arduino.write(dir, pwr, MotorDirection.MAINTAIN, 0.0)
        elif self.motor_state == AvailableMotors.LEFT:
            self.arduino.write(MotorDirection.MAINTAIN, 0.00, dir, pwr)


class ArduinoMotors:
    def __init__(self):
        self.arduino = ArduinoComms()
        self.heading = 0.0
        self.right_motor = ArduinoMotor(self.arduino, AvailableMotors.RIGHT)
        self.left_motor = ArduinoMotor(self.arduino, AvailableMotors.LEFT)

    def turn_right(self):
        self.right_motor.move(MotorDirection.FORWARD, 1) # idk if 1 works but thats what it be for now
        self.left_motor.move(MotorDirection.BACKWARD, 1) # idk if 1 works but thats what it be for now

    def turn_left(self):
        self.right_motor.move(MotorDirection.BACKWARD, 1) # idk if 1 works but thats what it be for now
        self.left_motor.move(MotorDirection.FORWARD, 1) # idk if 1 works but thats what it be for now

    def moveForwards(self, pwr):
        self.arduino.write(MotorDirection.FORWARD, pwr, MotorDirection.FORWARD, pwr)

    def stop(self):
        self.arduino.write(MotorDirection.STOP, 0.0, MotorDirection.STOP, 0.0)
