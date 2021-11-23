from driver.AS5600 import AS5600
from driver.motor import *
from VL53L0X import VL53L0X

import math

class Pendulum:

    def __init__(self):

        #init motor
        self.motor = MotorDriverTB6612FNG()

        #init distance sensor
        self.sensor_dist = VL53L0X(i2c_bus=1,i2c_address=0x29)
        self.sensor_dist.open()
        self.sensor_dist.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

        #init rotation sensor
        self.sensor_rot = AS5600()

        self.midPoint = 0
        self.zeroAngle = 0

        self.calibrated = False
        self.midpointCalibrated = False
        self.angleCalibrated = False

    def __del__(self):

        #stop motor
        motor.dc_motor_stop(0)

        #stop ranging
        self.sensor_dist.stop_ranging()
        self.sensor_dist.close()

    def setSpeed(self, speed):

        if speed < -200:
            speed = -200
            print("Motor speed set too high " + str(speed))
        elif speed > 200:
            speed = 200
            print("Motor speed set too high " + str(speed))
        else:
            return self.dc_motor_run(0, speed)

    def getPosition(self):
        '''Get pendulum position in distance from midpoint'''
        res = self.sensor_dist.get_distance() - self.midPoint
        return res

    def getRotation(self):
        '''Get pendulum angular distance to zero point'''
        angle_diff = self.sensor_rot.getAngleRadians() - self.zeroAngle()
        if angle_diff > math.pi/2.0:
            angle_diff -= math.pi
        elif angle_diff < -math.pi/2.0:
            angle_diff += math.pi

        return angle_diff

    def reset(self):
        #go to middle of track

        #wait until pendulum stands still

    def calibrate(self):
        self.calibrateMidpoint()
        self.calibrateZeroAngle()

    def calibrateMidpoint(self):
        '''calibrate the point where pendulum is in middle of the track'''

        self.setSpeed(-30)
        while self.
        self.calibrated = True

    def calibrateZeroAngle(self):
        '''calibrate the point where pendulum is in middle of the track'''
        print("Calibrating Zero Angle")
        print("Waiting for pendulum to stand still")
        self.reset()
        self.zeroAngle = self.sensor_rot.getAngleRadians()
        self.calibrated = True
        print("Zero angle calibrated")