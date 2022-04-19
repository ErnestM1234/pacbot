from sensorTesting import *

WALL_STOP_DIST = 50 # mm


class Directions(Enum):
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270

"""
class ArduinoMotors methods:

rotate_right()
rotate_left()
turn_right()
turn_left()

face_north()
face_south()
face_east()
face_west()

move_forwards()
move_backwards()
move_dist() ** currently unimplemented **
move_cells() ** currently unimplemented **
stop()

"""

class ArduinoMotors:
    def __init__(self):
        self.arduino = ArduinoComms()
        self.heading = 0
        self.odometer = 0

    # ------------------------ Rotations ------------------------ #
    """ rotate_right()
    input:  void
    return: void
    Rotate robot right
    """
    def rotate_right(self):
        self.arduino.write(MotorDirection.FORWARDS, 255, MotorDirection.BACKWARDS, 255)

    """ rotate_left()
    input:  void
    return: void
    Rotate robot left
    """
    def rotate_left(self):
        self.arduino.write(MotorDirection.BACKWARDS, 255, MotorDirection.FORWARDS, 255)

    # ------------------------ Turning ------------------------ #
    """ turn_right()
    input:  void
    return: void
    turns robot 90 degrees to the right, then stops
    """
    def turn_right(self):
        # todo: add right/left adjustment
        target_heading = (self.arduino.getHeading() + 90) % 360
        while (abs(target_heading - self.arduino.getHeading()) > 5):
            # todo: adjust 350 value and/or rework logic
            print("ArduinoMotors: turning right")
            # if (target_heading > 0 and self.arduino.getHeading() > 350): # handle case when robot's heading must pass 0 degrees
            #    target_heading -= 360
            self.rotate_right()
        self.stop()
    
    """ turn_left()
    input:  void
    return: void
    turns robot 90 degrees to the left, then stops
    """
    def turn_left(self):
        # todo: add right/left adjustment
        target_heading = self.arduino.getHeading() - 90
        while (abs(target_heading - self.arduino.getHeading()) > 5):
            # todo: adjust 350 value and/or rework logic
            print("ArduinoMotors: turning left")
            # if (target_heading < 0 and self.arduino.getHeading() < 10): # handle case when robot's heading must pass 0 degrees
            #    target_heading += 360
            self.rotate_left()
        self.stop()

    # ------------------------ Directions ------------------------ #
    """ face_north()
    input:  void
    return: void
    turns robot to face north (error = +/- 3 deg), then stops
    """
    def face_north(self):
        while True:
            # decide whether to go right or left
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
        while True:
            # decide whether to go right or left
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
            elif heading > Directions.EAST and heading < Directions.WEST:   # go right
                self.rotate_right()
            else:                                                           # go left
                self.rotate_left()
        self.stop()

    # ------------------------ Go/Stop ------------------------ #
    """ move_forwards()
    input:  void
    return: void
    moves robot forward at power 20 (note this does not check for obstacles)
    """
    def move_forwards(self):
        self.arduino.write(MotorDirection.FORWARDS, 255, MotorDirection.FORWARDS, 255)

    """ move_backwards()
    input:  void
    return: void
    moves robot backwards at power 20 (note this does not check for obstacles)
    """
    def move_backwards(self):
        self.arduino.write(MotorDirection.BACKWARDS, 255, MotorDirection.BACKWARDS, 255)

    """ move_dist()
    input:  dist - mm to move forwards
    return: void
    moves robot forwards specified distance in mm
    """
    def move_dist(self, dist):
        # this is a simulation implementation not real!!
        self.arduino.simulation_reset_odometer();
        while self.arduino.odometer < dist:
            self.move_forwards()

    """ move_cells()
    input:  dist - number of cells to move forwards
    return: void
    moves robot forwards specified distance in number of cells, then stops
    """
    def move_cells(self, dist):
        # keep in mind: each passage is 7" wide, there is a .25" boundary between each
        # this means we must move 7.25" or ~184 mm
        self.move_dist(dist * 184)
        self.stop()
    
    """ stop()
    input:  void
    return: void
    stops robot from moving
    """
    def stop(self):
        self.arduino.write(MotorDirection.FORWARDS, 0, MotorDirection.FORWARDS, 0)
