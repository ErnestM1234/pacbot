from sensorTesting import *

WALL_STOP_DIST = 50 # mm

# gyro tunings (for going straight)
KI_S = 1
KP_S = 1
KD_S = 1
# gyro tunings (for rotation)
KI_R = 1
KP_R = 1
KD_R = 1

KP_ANGLE = 3
KP_DISTANCE = 0

MAX_POWER = 180
MAX_POWER_DIFFERENTIAL = 50

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

MOTOR_SPEED = 200
DISTANCE_DIFF = 0 # distance differential
INIT_PWR_L = 150
INIT_PWR_R = 150

class ArduinoMotors:
    def __init__(self):
        self.arduino = ArduinoComms()
        self.heading = 0
        self.odometer = 0
        self.target_heading = 0

    # ------------------------ target heading ------------------------ #
    """ set_target_heading()
    input:  void
    return: void
    sets the target heading
    """
    def set_target_heading(self, target_heading):
        self.target_heading = target_heading

    """ rotate_to_target_heading()
    input:  void
    return: void
    rotates CW to the target heading
    """
    def rotate_to_target_heading(self):
        self.arduino.calibrate()

        pwr_left = 0
        pwr_right = 0
        # initial_heading = self.arduino.getHeading()
        while (abs(self.arduino.getHeading() - self.target_heading) < 3):
            current_heading = self.arduino.getHeading()
            
            mean_power = KP_ANGLE * (self.target_heading - current_heading)
            mean_power = min(mean_power, MAX_POWER)

            power_differential = KP_DISTANCE * (abs(self.arduino.readSensor("LEFT_ENCODER")) - abs(self.arduino.readSensor("RIGHT_ENCODER")))
            power_differential = min(power_differential, MAX_POWER_DIFFERENTIAL)
            power_differential = max(power_differential, -MAX_POWER_DIFFERENTIAL)

            pwr_left = mean_power - power_differential
            pwr_right = -mean_power - power_differential

            l_dir = MotorDirection.BACKWARDS # "backwards" (actually forwards)
            r_dir = MotorDirection.BACKWARDS

            if pwr_left < 0:
                l_dir = MotorDirection.FORWARDS
            if pwr_right < 0:
                r_dir = MotorDirection.FORWARDS
            
            self.arduino.write(l_dir, abs(l_dir), r_dir, abs(r_dir))

        
        # if self.arduino.getHeading() - self.target_heading < 0:
        #     target_heading
        self.arduino.write(MotorDirection.FORWARDS, MOTOR_SPEED, MotorDirection.BACKWARDS, MOTOR_SPEED)

    # ------------------------ Rotations ------------------------ #
    """ rotate_right()
    input:  void
    return: void
    Rotate robot right
    """
    def rotate_right(self):
        self.arduino.write(MotorDirection.FORWARDS, MOTOR_SPEED, MotorDirection.BACKWARDS, MOTOR_SPEED)

    """ rotate_left()
    input:  void
    return: void
    Rotate robot left
    """
    def rotate_left(self):
        self.arduino.write(MotorDirection.BACKWARDS, MOTOR_SPEED, MotorDirection.FORWARDS, MOTOR_SPEED)

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
            # print("ArduinoMotors: turning right")
            if abs(target_heading - self.arduino.getHeading()) < 180:
                self.face_north() # rotates right
            elif target_heading > self.arduino.getHeading():
                self.rotate_right()
            else:
                self.rotate_left()
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
            # print("ArduinoMotors: turning left")
            self.rotate_left()
        self.stop()

    # ------------------------ Directions ------------------------ #
    """ face_north()
    input:  void
    return: void
    turns robot to face north (error = +/- 3 deg), then stops
    """
    def face_north(self):
        # todo: prefer left or right of north
        while True:
            # decide whether to go right or left
            heading = self.arduino.getHeading()
            if heading < 0 + 3 and heading >  0 + 360 - 3: # stop with error +/- 3 deg
                break
            elif heading > 180 and heading < 360: # go right
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
            if heading < 270 + 3 and heading >  270 + 360 - 3: # stop with error +/- 3 deg
                break
            elif heading > 0 and heading < 270: # go right
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
        self.arduino.write(MotorDirection.FORWARDS, MOTOR_SPEED, MotorDirection.FORWARDS, MOTOR_SPEED)

    """ move_backwards()
    input:  void
    return: void
    moves robot backwards at power 20 (note this does not check for obstacles)
    """
    def move_backwards(self):
        self.arduino.write(MotorDirection.BACKWARDS, MOTOR_SPEED, MotorDirection.BACKWARDS, MOTOR_SPEED)

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
