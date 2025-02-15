from driver.AS5600 import AS5600
from driver.motor import *
from VL53L0X import *

import math
#import numpy as np
#from numpy_ringbuffer import RingBuffer
from simple_pid import PID
import time
import threading

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

        self.currRawAngle = 0.0
        self.currRawPosition = 0.0
        self.stateCartPosition = 0.0
        self.stateCartVelocity = 0.0
        self.statePendulumPosition = 0.0
        self.statePendulumVelocity = 0.0
        self.max_motor_command = 255.0
        #self.rbCartPosition = RingBuffer(capacity=5, dtype=np.float32)
        #self.rbPendulumPosition = RingBuffer(capacity=5, dtype=np.float32)
        print("finished init6")
        #init PID controller for moving pendulum to middle point
        self.midPoint = 250
        self.midPoint_offset = 30.0 #offset of middle of mount to the reflector
        self.pid = PID (1.3, 0.01, 0.0, setpoint = self.midPoint, sample_time=0.05, output_limits=(-120,120))
        print("finished init7")

        self.threadMeasure = threading.Timer(0.01, self.measure)
        self.threadMeasure.start()


    def __del__(self):
        self.threadMeasure.cancel()

        #stop motor
        self.motor.dc_motor_stop(0)

        #stop ranging
        self.sensor_dist.stop_ranging()
        self.sensor_dist.close()

    def measure(self):
        tmpAngle = self.sensor_rot.getAngleRadians()
        tmpPosition = self.sensor_dist.get_distance()

        self.currRawAngle = tmpAngle
        self.currRawPosition = tmpPosition

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
            self.motor.dc_motor_run(0, speed)
            return

    def getPosition(self):
        '''Get pendulum position in distance from midpoint'''
        res = self.currRawPosition - self.midPoint
        return res

    def getRotation(self):
        '''Get pendulum angular distance to zero point'''
        angle_diff = normalize_angle(self.currRawAngle - self.zeroAngle)
        return angle_diff

    def reset(self):
        if not self.calibrated:
            self.calibrate()
        #go to middle of track
        self.moveToMidPoint()
	self.waitUntilPendulumStopped()

    def waitUntilPendulumStopped(self):
        moving = True
        angle = 0
        angle_threshold = np.radians(5) # set threshold to 5 degrees
        while moving:
            curr_angle = self.sensor_rot.getAngleRadians()
            time.sleep(0.1)
            if np.fabs(normalize_angle(curr_angle-angle)) < angle_threshold:
                moving = False
                self.zeroAngle = curr_angle
            angle = curr_angle

    def calibrate(self):
        self.calibrateMidpoint()
        self.calibrateZeroAngle()
        if self.midpointCalibrated and self.angleCalibrated: self.calibrated = True

    def calibrateMidpoint(self):
        '''calibrate the point where pendulum is in middle of the track'''

        self.setSpeed(70)
        time.sleep(6)
        max_dist=self.sensor_dist.get_distance()
        print("max dist ", max_dist)

        self.setSpeed(-70)
        time.sleep(6)
        min_dist=self.sensor_dist.get_distance()
        print("min dist ", max_dist)

        self.midPoint=(max_dist+min_dist)/2.+self.midPoint_offset
        self.midpointCalibrated = True

    def calibrateZeroAngle(self):
        '''calibrate the point where pendulum is in middle of the track'''
        print("Calibrating Zero Angle")
        print("Waiting for pendulum to stand still")
        #self.reset()
        self.waitUntilPendulumStopped()
        self.zeroAngle = self.sensor_rot.getAngleRadians()
        self.angleCalibrated = True
        print("Zero angle calibrated")

    def isCalibrated(self):
        return self.calibrated

    def moveToMidPoint(self):

        goalReached = False

        while not goalReached:
            dist = self.sensor_dist.get_distance()
            if abs(dist - self.midPoint) < 5:
                self.motor.dc_motor_stop(0)
                return
            speed = self.pid(dist)
            if speed < 0 and speed >  -60:
                speed = -60
            elif speed < 0 and speed >  -60:
                speed = -60
            elif speed > 0 and speed > 60:
                speed = 60
            elif speed > 0 and speed < 60:
                speed = 60

            self.setSpeed(speed)
            print("Distance, Speed " + str(dist) + " " + str(speed))
            time.sleep(0.1)

def normalize_angle(x):
    return ((x + np.pi) % (2 * np.pi)) - np.pi

if __name__ == '__main__':

    pendulum = Pendulum()
    pendulum.calibrateMidpoint()
    pendulum.moveToMidPoint()
