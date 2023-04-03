#!/bin/bash
import smbus2 as smbus
import time
import math

class AS5600:

    _address = 0x36;
    
    _zmco = 0x00
    _zpos_hi = 0x01
    _zpos_lo = 0x02
    _mpos_hi = 0x03
    _mpos_lo = 0x04
    _mang_hi = 0x05
    _mang_lo = 0x06
    _conf_hi = 0x07
    _conf_lo = 0x08
    _raw_ang_hi = 0x0c
    _raw_ang_lo = 0x0d
    _ang_hi = 0x0e
    _ang_lo = 0x0f
    _stat = 0x0b
    _agc = 0x1a
    _mag_hi = 0x1b
    _mag_lo = 0x1c
    _burn = 0xff

    _raw_offset = 0

    def __init__(self):
        self.bus = smbus.SMBus(1)

    def setOffset(self, offset):
        self._raw_offset = offset

    def rawAngleToDegrees(self,rawAngle):
        #Raw data reports 0 - 4095 segments, which is 0.087 of a degree
        res = rawAngle * 0.087
        return res

    def rawAngleToRadians(self,rawAngle):
        #Raw data reports 0 - 4095 segments, which is 0.087 of a degree
        res = rawAngle * 0.087 * math.pi / 180.0
        return res

    def writeTwoBytes(self, start_address, data):
        self.bus.write_i2c_block_data(self._address, start_address, data)

    def readTwoBytes(self, start_address):
        [high, low] = self.bus.read_i2c_block_data(self._address, start_address, 2)
        high = high << 8
        res = high | low
        return res
        
    def getRawAngle(self):
        return self.readTwoBytes(self._raw_ang_hi)

    def getAngleDegrees(self):
        return self.rawAngleToDegrees(self.getRawAngle())

    def getCorrAngleRadians(self):
        corr_raw_angle = (self.getRawAngle() + offset) % 4095
        return self.rawAngleToRadians(corr_raw_angle)

    def getCorrAngleDegrees(self):
        corr_raw_angle = (self.getRawAngle() + offset) % 4095
        return self.rawAngleToDegrees(corr_raw_angle)


    def getMaxAngle(self):
        return self.readTwoBytes(self._mang_hi)
    
    def getScaledAngle(self):
        return self.readTwoBytes(self._ang_hi)

    def getEndPosition(self):
        return self.readTwoBytes(self._mpos_hi)

    def getStartPosition(self):
        return self.readTwoBytes(self._zpos_hi)

    def getMagnitude(self):
        return self.readTwoBytes(self._mag_hi)

    def setEndPosition(self, endAngle=-1):

        if (endAngle == -1):
            _rawEndAngle = self.getRawAngle();
        else:
            _rawEndAngle = endAngle;

        byteAngle = (_rawEndAngle).to_bytes(2, byteorder='big') 
        print ("byte angle")
        print(byteAngle)
        self.writeTwoBytes(self._mpos_hi, byteAngle);
        time.sleep(2);
     
        return (self.getEndPosition());

    def setStartPosition(self, startAngle=-1):

        if (startAngle == -1):
            _rawStartAngle = self.getRawAngle();
        else:
            _rawStartAngle = startAngle;

        byteAngle = (_rawStartAngle).to_bytes(2, byteorder='big') 
        print ("byte angle")
        print(byteAngle)
        self.writeTwoBytes(self._zpos_hi, byteAngle);
        time.sleep(2);
     
        return (self.getEndPosition());

    def setMaxAngle(self, maxAngle=-1):

        if (maxAngle == -1):
            _rawMaxAngle = self.getRawAngle();
        else:
            _rawMaxAngle = maxAngle;

        byteAngle = (_rawMaxAngle).to_bytes(2, byteorder='big') 
        self.writeTwoBytes(self._mang_hi, byteAngle);
        time.sleep(2);
     
        return (self.getMaxAngle());

    def detectMagnet(self):

        retVal = 0
        magStatus = 0
        #0 0 MD ML MH 0 0 0
        #MD high = magnet detected
        #ML high = AGC Maximum overflow, magnet too weak
        #MH high = AGC minimum overflow, magnet too strong
        magStatus = self.bus.read_byte_data(self._address,self._stat)

        print(bin(magStatus))
        if (magStatus & 0x20):
            retVal = 1

        if retVal: print("Magnet detected")
        else: print("Magnet not detected")
        return retVal;

    def getMagnetStrength(self):

        retVal = 0
        magStatus = 0
        #0 0 MD ML MH 0 0 0
        #MD high = magnet detected
        #ML high = AGC Maximum overflow, magnet too weak
        #MH high = AGC minimum overflow, magnet too strong
        magStatus = self.bus.read_byte_data(self._address,self._stat)

        print(bin(magStatus))
        if (magStatus & 0x20):
            retVal = 2
            print("Magnet strength ok")
            if (magStatus & 0x10):
                retVal = 1 #too weak
                print("Magnet too weak")
            elif (magStatus & 0x08):
                retVal = 3 #to strong
                print("Magnet too strong")
        return retVal



if __name__ == '__main__':
    
    sensor = AS5600()

    print(sensor.getRawAngle())
    print(sensor.getMaxAngle())
    print(sensor.getScaledAngle())
    print(sensor.readTwoBytes(0x0c))    
    print(sensor.detectMagnet())
    print(sensor.getMagnetStrength())
    #print(sensor.setStartPosition())
    print(sensor.getStartPosition())

    offset = 4095 - sensor.getRawAngle()
    sensor.setOffset(offset)
    while True:
        print("raw",sensor.getRawAngle())
        #print("raw",sensor.getAngleDegrees())
        print("cor",sensor.getCorrAngleDegrees())
        #print(sensor.getAngleRadians())



    
