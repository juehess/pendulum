from driver.AS5600 import AS5600
from driver.motor import *
from VL53L0X import *

import math
#import numpy as np
#from numpy_ringbuffer import RingBuffer
from simple_pid import PID

class Pendulum:

    def __init__(self):

        #init motor
        self.motor = MotorDriverTB6612FNG()
        print("finished init1")
        #init distance sensor
        self.sensor_dist = VL53L0X(i2c_bus=1,i2c_address=0x29)
        self.sensor_dist.open()
        self.sensor_dist.start_ranging(Vl53l0xAccuracyMode.BETTER)
        print("finished init2")
        #init rotation sensor
        self.sensor_rot = AS5600()
        print("finished init3")
        self.midPoint = 0
        self.zeroAngle = 0
        print("finished init4")
        self.calibrated = False
        self.midpointCalibrated = False
        self.angleCalibrated = False
        print("finished init5")
        self.stateCartPosition = 0
        self.stateCartVelocity = 0
        self.statePendulumPosition = 0
        self.statePendulumVelocity = 0
        #self.rbCartPosition = RingBuffer(capacity=5, dtype=np.float32)
        #self.rbPendulumPosition = RingBuffer(capacity=5, dtype=np.float32)
        print("finished init6")
        #init PID controller for moving pendulum to middle point
        self.midPoint = 280
        self.pid = PID (1.2, 0.05, 0.0, setpoint = self.midPoint, sample_time=0.05, output_limits=(-60,60))
        print("finished init7")

    def __del__(self):

        #stop motor
        self.motor.dc_motor_stop(0)

        #stop ranging
        self.sensor_dist.stop_ranging()
        self.sensor_dist.close()

    def computeState(self):

        pos = self.getPosition()
        #self.rbCartPosition.append(pos)
        rot = self.getRotation()
        #self.rbPendulumPosition.append(rot)

    def getState(self):
        return self.stateCartPosition, self.stateCartVelocity, self.statePendulumPosition, self.statePendulumVelocity

    def setSpeed(self, speed):

        if speed < -200:
            speed = -200
            print("Motor speed set too high " + str(speed))
        elif speed > 200:
            speed = 200
            print("Motor speed set too high " + str(speed))
        else:
            return self.motor.dc_motor_run(0, speed)

    def getPosition(self):
        '''Get pendulum position in distance from midpoint'''
        res = self.sensor_dist.get_distance() - self.midPoint
        return res

    def getRotation(self):
        '''Get pendulum angular distance to zero point'''
        angle_diff = self.sensor_rot.getAngleRadians() - self.zeroAngle
        if angle_diff > math.pi/2.0:
            angle_diff -= math.pi
        elif angle_diff < -math.pi/2.0:
            angle_diff += math.pi

        return angle_diff

    #def reset(self):
    ##    #go to middle of track
#
#        #wait until pendulum stands still

    def calibrate(self):
        self.calibrateMidpoint()
        self.calibrateZeroAngle()
        if self.midpointCalibrated and self.angleCalibrated: self.calibrated = True

    def calibrateMidpoint(self):
        '''calibrate the point where pendulum is in middle of the track'''

        self.setSpeed(60)
        time.sleep(5)
        max_dist=self.sensor_dist.get_distance()

        self.setSpeed(-60)
        time.sleep(5)
        min_dist=self.sensor_dist.get_distance()

        self.midPoint=max_dist-min_dist
        self.midpointCalibrated = True

    def calibrateZeroAngle(self):
        '''calibrate the point where pendulum is in middle of the track'''
        print("Calibrating Zero Angle")
        print("Waiting for pendulum to stand still")
        #self.reset()
        self.zeroAngle = self.sensor_rot.getAngleRadians()
        self.calibrated = True
        print("Zero angle calibrated")

    def moveToMidPoint(self):

        goalReached = False

        while not goalReached:
            dist = self.sensor_dist.get_distance()
            speed = self.pid(dist)
            print(speed)
            if speed < 0 and speed >  -30:
                speed = -30
            elif speed > 0 and speed < 30:
                speed = 30
            self.setSpeed(speed)
            print("Distance, Speed " + str(dist) + " " + str(speed))
            time.sleep(0.1)

if __name__ == '__main__':

    pendulum = Pendulum()
    pendulum.moveToMidPoint()