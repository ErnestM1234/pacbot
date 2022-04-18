from sensorTesting import *

WALL_STOP_DIST = 5 # cm


class ArduinoMotors:
    def __init__(self):
        self.arduino = ArduinoComms()
        self.heading = 0.0

    """ turn_right()
    input:  void
    return: void
    Rotate robot right
    """
    def turn_right(self):
        self.arduino.write(MotorDirection.FORWARDS, 20, MotorDirection.BACKWARDS, 20)

    """ turn_left()
    input:  void
    return: void
    Rotate robot left
    """
    def turn_left(self):
        self.arduino.write(MotorDirection.BACKWARDS, 20, MotorDirection.FORWARDS, 20)

    """ moveForwards()
    input:  void
    return: void
    moves robot forward at power 20
    """
    def moveForwards(self):
        self.arduino.write(MotorDirection.FORWARD, 20, MotorDirection.FORWARD, 20)

    """ stop()
    input:  void
    return: void
    stops robot from moving
    """
    def stop(self):
        self.arduino.write(MotorDirection.STOP, 0, MotorDirection.STOP, 0)
