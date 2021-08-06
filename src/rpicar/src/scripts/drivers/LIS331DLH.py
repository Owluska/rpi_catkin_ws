#! /usr/bin/env python3
import smbus
from time import sleep


class LIS331DLH():

    SLAVE_ADDRESS = 0x18 
    SLAVE_ADDRESS_ALT = 0x19 

    BASE_IMU_CTRL_REG1 = 0x20 
    BASE_IMU_CTRL_REG2 = 0x21 
    BASE_IMU_CTRL_REG3 = 0x22
    BASE_IMU_CTRL_REG4 = 0x23
    BASE_IMU_CTRL_REG5 = 0x24

    BASE_IMU_OUT_X_L = 0x28
    BASE_IMU_OUT_X_H = 0x29
    BASE_IMU_OUT_Y_L = 0x2A
    BASE_IMU_OUT_Y_H = 0x2B
    BASE_IMU_OUT_Z_L = 0x2C
    BASE_IMU_OUT_Z_H = 0x2D

    CTRL_REG4_FS0 = 0x10 
    CTRL_REG4_FS1 = 0x20 

    CTRL_REG1_X_EN = 0x01 
    CTRL_REG1_Y_EN = 0x02 
    CTRL_REG1_Z_EN = 0x04 
    CTRL_REG1_PM0 = 0x20 
    CTRL_REG1_PM1 = 0x40 
    CTRL_REG1_PM2 = 0x80 

    
    GRAVITY_EARTH = 9.8

    SENS_2G = 1 * 4 / (2 ** 16)
    SENS_4G = 2 * 4 / (2 ** 16)
    SENS_8G = 3.9 * 4 /(2 ** 16)

    ACC_RANGES = {'RANGE_2G':1, 'RANGE_4G':2, 'RANGE_8G':3}

    def __init__(self, channel = 0):
        self.bus = smbus.SMBus(1) # start comm with i2c bus

        self.scale_factor = 1.0 
        
        self.data = self.CTRL_REG1_X_EN | self.CTRL_REG1_Y_EN | self.CTRL_REG1_Z_EN
        self.data |= self.CTRL_REG1_PM0
        self.bus.write_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_CTRL_REG1, self.data)
        self.set_range(self.ACC_RANGES['RANGE_2G'])

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

    
    def set_range(self, range):
        self.data = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_CTRL_REG4)
        print(self.data)
        self.data &= ~(self.CTRL_REG4_FS0 | self.CTRL_REG4_FS1 ) & 0xFF
        print(self.data)

        if range == self.ACC_RANGES['RANGE_2G']:
            self.scale_factor = self.SENS_2G
        elif range == self.ACC_RANGES['RANGE_4G']:
            self.scale_factor = self.SENS_4G
        elif range == self.ACC_RANGES['RANGE_8G']:
            self.scale_factor = self.SENS_8G
        else:
            self.scale_factor = self.SENS_2G
        self.bus.write_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_CTRL_REG4, self.data)

    def acc_sleep(toSleep):
        data = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_CTRL_REG1)
        if toSleep:
            data &= ~self.CTRL_REG1_PM0 & 0xFF
        else:
            data |= self.CTRL_REG1_PM0
        self.bus.write_byte(self.BASE_IMU_CTRL_REG1, data)

    def readXYZ(self):
        data = [0, 0, 0, 0, 0, 0]
        data = self.bus.read_i2c_block_data(self.SLAVE_ADDRESS, 0x08 | self.BASE_IMU_OUT_X_L)
        print(data)
        x = data[1] << 8 | data[0]
        y = data[3] << 8 | data[2]
        z = data[5] << 8 | data[4]
        return z, y, z
    
    def readX(self):
        XL = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_OUT_X_L)
        XH = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_OUT_X_H)
        return XH << 8 | XL

    def readY(self):
        YL = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_OUT_Y_L)
        YH = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_OUT_Y_H)
        return YH << 8 | YL
    
    def readZ(self):
        ZL = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_OUT_Z_L)
        ZH = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.BASE_IMU_OUT_Z_H) 
        return ZH << 8 | ZL

    def read_acc_gXYZ(self):
        x = self.readX()
        y = self.readY()
        z = self.readZ()

        self.ax = x * self.scale_factor
        self.ay = y * self.scale_factor
        self.az = z * self.scale_factor

    def read_acc_aXYZ(self):
        self.read_acc_gXYZ()
        self.ax *= self.GRAVITY_EARTH 
        self.ay *= self.GRAVITY_EARTH
        self.az *= self.GRAVITY_EARTH


 


acc = LIS331DLH()
acc.read_acc_gXYZ()
# print(acc.scale_factor)
print(acc.ax, acc.ay, acc.az)