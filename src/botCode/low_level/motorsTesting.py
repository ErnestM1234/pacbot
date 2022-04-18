from sensorTesting import *

WALL_STOP_DIST = 5 # cm


class Directions(Enum):
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270

"""
ArduinoMotors methods:

rotate_right()
rotate_left()
turn_right()
turn_left()

faceNorth()
faceSouth()
faceEast()
faceWest()

moveForwards()
stop()


"""

class ArduinoMotors:
    def __init__(self):
        self.arduino = ArduinoComms()
        self.heading = 0.0

    """ rotate_right()
    input:  void
    return: void
    Rotate robot right
    """
    def rotate_right(self):
        self.arduino.write(MotorDirection.FORWARDS, 20, MotorDirection.BACKWARDS, 20)

    """ rotate_left()
    input:  void
    return: void
    Rotate robot left
    """
    def rotate_left(self):
        self.arduino.write(MotorDirection.BACKWARDS, 20, MotorDirection.FORWARDS, 20)

    """ turn_right()
    input:  void
    return: void
    turns robot 90 degrees to the right, then stops
    """
    def turn_right(self):
        old_heading = self.arduino.getHeading()
        while abs(old_heading - self.arduino.getHeading()) > 90:
            self.rotate_right()
        self.stop()
    
    """ turn_left()
    input:  void
    return: void
    turns robot 90 degrees to the left, then stops
    """
    def turn_right(self):
        old_heading = self.arduino.getHeading()
        while abs(old_heading - self.arduino.getHeading()) < 90:
            self.rotate_left()
        self.stop()


    """ face_north()
    input:  void
    return: void
    turns robot to face north (error = +/- 3 deg), then stops
    """
    def face_north(self):
        # decide whether to go right or left
        while True:
            heading = self.arduino.getHeading()
            if heading < Directions.NORTH + 3 and heading >  Directions.NORTH + 360 - 3: # stop with error +/- 3 deg
                break
            elif heading > Directions.SOUTH and heading < Directions.NORTH: # go right
                self.rotate_right()
            else:                                                           # go left
                self.rotate_left()
        self.stop()

    """ face_south()
    input:  void
    return: void
    turns robot to face north (error = +/- 3 deg), then stops
    """
    def face_south(self):
        # decide whether to go right or left
        while True:
            heading = self.arduino.getHeading()
            if heading < Directions.SOUTH + 3 and heading >  Directions.SOUTH + 360 - 3: # stop with error +/- 3 deg
                break
            elif heading > Directions.NORTH and heading < Directions.SOUTH: # go right
                self.rotate_right()
            else:                                                           # go left
                self.rotate_left()
        self.stop()

    """ face_east()
    input:  void
    return: void
    turns robot to face north (error = +/- 3 deg), then stops
    """
    def face_east(self):
        # decide whether to go right or left
        while True:
            heading = self.arduino.getHeading()
            if heading < Directions.EAST + 3 and heading >  Directions.EAST + 360 - 3: # stop with error +/- 3 deg
                break
            elif heading < Directions.WEST and heading > Directions.EAST:   # go left
                self.rotate_left()
            else:                                                           # go right
                self.rotate_right()
        self.stop()

    """ face_west()
    input:  void
    return: void
    turns robot to face north (error = +/- 3 deg), then stops
    """
    def face_west(self):
        # decide whether to go right or left
        while True:
            heading = self.arduino.getHeading()
            if heading < Directions.WEST + 3 and heading >  Directions.WEST + 360 - 3: # stop with error +/- 3 deg
                break
            elif heading > Directions.EAST and heading < Directions.WEST: # go right
                self.rotate_right()
            else:                                                           # go left
                self.rotate_left()
        self.stop()

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
