from motor import Motor, MotorDirection
from encoders import Encoder
from pins import *
from sensors import Sensors
from GPIOhelpers import *
from PID import *
setGPIO()

MOTOR_SPEED = 5
TICKS_CELL = 500
TICKS_TURN = 200
WALL_THRESHOLD_DIAG = 120
WALL_DISTANCE_DIAG = 70
class Motors:
    def __init__(self):
        self.sensors = Sensors([pins.tof_front,pins.tof_rear,pins.tof_fleft,pins.tof_fright,pins.tof_rleft,pins.tof_rright], ["front", "rear","fleft","fright","rleft","rright"], [0x30,0x31,0x32,0x33,0x34,0x35])
        self._frontIR = self.sensors.sensors["front"]
        self._fleftIR = self.sensors.sensors["fleft"]
        self._frightIR = self.sensors.sensors["fright"]
        self._rearIR = self.sensors.sensors["rear"]
        self._rleftIR = self.sensors.sensors["rleft"]
        self._rrightIR = self.sensors.sensors["rright"]

        self.dir = True

        self.encoderLeft = Encoder(pins.encoder_pin_l1, pins.encoder_pin_l2)
        self.encoderRight = Encoder(pins.encoder_pin_r1, pins.encoder_pin_r2)

        self.left_motor = Motor("Left", pins.motor_speed_l, pins.motor_direction_l, 0)

        self.right_motor = Motor("Right", pins.motor_speed_r, pins.motor_direction_r, 0)

        self.setpointL = 0
        self.inputL = 0
        self.PIDLeft = PID(self.inputL, self.setpointL, 2.0, 0.002, 0.000, DIRECT, Timer)
        self.PIDLeft.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDLeft.set_mode(AUTOMATIC)

        self.setpointR = 0
        self.inputR = 0
        self.PIDRight = PID(self.inputR, self.setpointR, 2.0, 0.002, 0.000, DIRECT, Timer)
        self.PIDRight.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDRight.set_mode(AUTOMATIC)

        self.setpointfL = 0
        self.inputfL = 0
        self.PIDfLeft = PID(self.inputfL, self.setpointfL, 2.0, 0.002, 0.000, DIRECT, Timer)
        self.PIDfLeft.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDfLeft.set_mode(AUTOMATIC)
        
        self.setpointfR = 0
        self.inputfR = 0
        self.PIDfRight = PID(self.inputfR, self.setpointfR, 2.0, 0.002, 0.000, DIRECT, Timer)
        self.PIDfRight.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDfRight.set_mode(AUTOMATIC)

        self.setpointrL = 0
        self.inputrL = 0
        self.PIDrLeft = PID(self.inputrL, self.setpointrL, 2.0, 0.002, 0.000, DIRECT, Timer)
        self.PIDrLeft.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDrLeft.set_mode(AUTOMATIC)
        
        self.setpointrR = 0
        self.inputrR = 0
        self.PIDrRight = PID(self.inputrR, self.setpointrR, 2.0, 0.002, 0.000, DIRECT, Timer)
        self.PIDrRight.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDrRight.set_mode(AUTOMATIC)

    def read_encoders(self):
        if self.dir:
            return (self.encoderLeft.read(), self.encoderRight.read())
        else:
            return (-self.encoderLeft.read(), -self.encoderRight.read())

    def raw_encoders(self):
        return (self.encoderLeft.read(), self.encoderRight.read())

    def reverse_direction(self):
        self.dir = not self.dir

    def move_motors(self, left, right):
        if left >= 0:
            self.left_motor.move(MotorDirection.FORWARD, abs(left))
        else:
            self.left_motor.move(MotorDirection.BACKWARD, abs(left))
        if right >= 0:
            self.right_motor.move(MotorDirection.FORWARD, abs(right))
        else:
            self.right_motor.move(MotorDirection.BACKWARD, abs(right))

    def stop(self):
        print("stopped")
        self.right_motor.stop()
        self.left_motor.stop()

    def wait(self, ms):
        delay(ms/1000)

    def move_ticks(self, ticks_l, ticks_r):
        self.encoderLeft.write(0)
        self.encoderRight.write(0)

        self.setpointfL = ticks_l
        self.setpointfR = ticks_r

        self.inputfR = 0
        self.inputfL = 0

        time = self.PIDRight.millis()

        while ((abs(self.inputL - self.setpointL) > 10 or abs(self.inputR - self.setpointR) > 10) and (self.PIDRight.millis() - time < 2000)):
            self.inputfL , self.inputfR = self.read_encoders()

            self.PIDfRight.compute(self.inputfR, self.setpointfR)
            self.PIDfLeft.compute(self.inputfL, self.setpointfL)

            l_rem = abs(self.inputfL - self.setpointfL)
            r_rem = abs(self.inputfR - self.setpointfR)

            if l_rem > r_rem :
                self.move_motors(self.PIDLeft.output(),0)
                print("left")
            elif r_rem > l_rem:
                self.move_motors(0,self.PIDRight.output())
                print("right")
            else:
                self.move_motors(self.PIDLeft.output(),self.PIDRight.output()) 
                print("both")
        self.stop()


    def forward(self):
        self.PIDfLeft.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDfRight.set_output_limits(-1 * MOTOR_SPEED, MOTOR_SPEED)
        self.PIDfLeft.set_tunings(0.6,0.01,0.01)
        self.PIDfRight.set_tunings(0.6,0.01,0.01)

        self.move_ticks(TICKS_CELL, TICKS_CELL)

    def advance(self, ticks):
        self.encoderLeft.write(0)
        self.encoderRight.write(0)

        distance_l, distance_r = self.read_encoders()

        while (distance_l + distance_r < ticks * 2):
            if (self._frightIR.get_distance() < WALL_THRESHOLD_DIAG and self._fleftIR.get_distance() < WALL_THRESHOLD_DIAG):
                self.followFront()
            elif (self._rrightIR.get_distance() < WALL_THRESHOLD_DIAG and self._rleftIR.get_distance() < WALL_THRESHOLD_DIAG):
                self.followRear()
            elif (self._fleftIR.get_distance() <  WALL_THRESHOLD_DIAG and self._rleftIR.get_distance() <  WALL_THRESHOLD_DIAG):
                self.followLeft()
            elif (self._frightIR.get_distance() <  WALL_THRESHOLD_DIAG and self._rrightIR.get_distance() <  WALL_THRESHOLD_DIAG):
                self.followRight()
            else:
                print("move")
                self.move_ticks(ticks - abs(distance_l), ticks - abs(distance_r))

        offset = (distance_l - distance_r)/2
        self.move_ticks(-1 * offset, offset)
        self.stop()

    def turn_around_l():
        self.turn_left()
        self.turn_left()

    def turn_around_r():
        self.turn_right()
        self.turn_right()

    def turn_left(self): 
        # if self._frontIR.get_distance() < WALL_THRESHOLD:
        #     self.front_align()
        self.PIDfLeft.set_tunings(1.7,0.01,0)
        self.PIDfRight.set_tunings(1.7,0.01,0)
        self.move_ticks(-1 * TICKS_TURN, TICKS_TURN)

    def turn_right(self):
        # if self._frontIR.get_distance() < WALL_THRESHOLD:
        #     self.front_align()
        self.PIDfLeft.set_tunings(1.7,0.01,0)
        self.PIDfRight.set_tunings(1.7,0.01,0)
        self.move_ticks(TICKS_TURN, -1 * TICKS_TURN)

    # def front_align():

    def followFront(self):
        print("fFront")
        self.PIDfLeft.set_tunings(0.6,0.01,0.01)
        self.PIDfRight.set_tunings(0.6,0.01,0.01)
        self.setpointfR = WALL_DISTANCE_DIAG
        self.setpointfL = WALL_DISTANCE_DIAG

        self.inputfR = self._frightIR.get_distance()
        self.inputfL = self._fleftIR.get_distance()

        self.PIDfRight.compute(self.inputfR, self.setpointfR)
        self.PIDfLeft.compute(self.inputfL, self.setpointfL)

        if self.dir:
            self.move_motors((MOTOR_SPEED + self.PIDfLeft.output())/2, (MOTOR_SPEED + self.PIDfRight.output())/2)
        else: 
            self.move_motors(-(MOTOR_SPEED + self.PIDfLeft.output())/2, -(MOTOR_SPEED + self.PIDfRight.output())/2)

        

    def followRear(self):
        print("fRear")

        self.PIDrLeft.set_tunings(0.6,0.01,0.01)
        self.PIDrRight.set_tunings(0.6,0.01,0.01)
        self.setpointrR = WALL_DISTANCE_DIAG
        self.setpointrL = WALL_DISTANCE_DIAG

        self.inputrR = self._rrightIR.get_distance()
        self.inputrL = self._rleftIR.get_distance()
        print(self.inputrR)
        print(self.inputrL)


        self.PIDrRight.compute(self.inputrR, self.setpointrR)
        self.PIDrLeft.compute(self.inputrL, self.setpointrL)

        if self.dir:
            self.move_motors((MOTOR_SPEED + self.PIDrLeft.output())/2, (MOTOR_SPEED + self.PIDrRight.output())/2)
        else:
            self.move_motors(-(MOTOR_SPEED + self.PIDrLeft.output())/2, -(MOTOR_SPEED + self.PIDrRight.output())/2)

    def followLeft(self):
        print("fLeft")

        self.PIDfLeft.set_tunings(0.6,0.01,0.01)
        self.PIDrLeft.set_tunings(0.6,0.01,0.01)
        self.setpointfL = WALL_DISTANCE_DIAG
        self.setpointrL = WALL_DISTANCE_DIAG


        self.inputfL = self._flefttIR.get_distance()
        self.inputrL = self._rleftIR.get_distance()

        self.PIDfLeft.compute(self.inputfL, self.setpointfL)
        self.PIDrLeft.compute(self.inputrL, self.setpointrL)

        if self.dir:
            self.move_motors((MOTOR_SPEED + self.PIDfLeft.output())/2, (MOTOR_SPEED + self.PIDrLeft.output())/2)
        else:
            self.move_motors(-(MOTOR_SPEED + self.PIDfLeft.output())/2, -(MOTOR_SPEED + self.PIDrLeft.output())/2)



    def followRight(self):
        print("fRight")

        self.PIDfRight.set_tunings(0.6,0.01,0.01)
        self.PIDrRight.set_tunings(0.6,0.01,0.01)
        self.setpointfR = WALL_DISTANCE_DIAG
        self.setpointrR = WALL_DISTANCE_DIAG

        self.inputfR = self._frightIR.get_distance()
        self.inputrR = self._rrightIR.get_distance()              
            
        self.PIDfRight.compute(self.inputfR, self.setpointfR)
        self.PIDrRight.compute(self.inputrR, self.setpointrR)

        if self.dir:
            self.move_motors((MOTOR_SPEED + self.PIDfRight.output())/2, (MOTOR_SPEED + self.PIDrRight.output())/2)
        else:
            self.move_motors(-(MOTOR_SPEED + self.PIDfRight.output())/2, -(MOTOR_SPEED + self.PIDrRight.output())/2)

M = Motors()
#M.reverse_direction()
M.advance(1000)
M.stop()
print("done")

import sys
sys.exit()







