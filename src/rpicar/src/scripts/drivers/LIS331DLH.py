#! /usr/bin/env python3
import smbus
from time import sleep
import struct

class LIS331DLH():
    #Accelerometer address
    SLAVE_ADDRESS = 0x18 
    #Registers addresses
    CTRL_REG1 = 0x20 
    CTRL_REG2 = 0x21 
    CTRL_REG3 = 0x22
    CTRL_REG4 = 0x23
    CTRL_REG5 = 0x24

    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_H = 0x2A
    OUT_Y_H = 0x2B
    OUT_Z_L = 0x2C
    OUT_Z_L = 0x2D

    STATUS_REG = 0x27
    #Bytes
    FS0 = 0x10 
    FS1 = 0x20 

    X_EN = 0x01 
    Y_EN = 0x02 
    Z_EN = 0x04 
    
    PM0 = 0x20 
    PM1 = 0x40 
    PM2 = 0x80 
    
    ZYXOR = 0x80
    
    XDA = 0x01
    YDA = 0x02
    ZDA = 0x04

    ZYXDA = 0x08

    BDU = 0x08
    
    GRAVITY_EARTH = 9.80665

    SENS_2G = 2 * 2/ (2 ** 16)
    SENS_4G = 4 * 2/ (2 ** 16)
    SENS_8G = 8 * 2/ (2 ** 16)

    ACC_RANGES = {'RANGE_2G':SENS_2G, 'RANGE_4G':SENS_4G, 'RANGE_8G':SENS_8G}

    def __init__(self, channel = 0):
        self.bus = smbus.SMBus(1) # start comm with i2c bus

        self.scale_factor = 1.0 
        
        self.setup()

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

    
    def setup(self):
        #enable x, y, z axes
        self.data = self.X_EN | self.Y_EN | self.Z_EN
        # power mode - normal
        self.data |= self.PM0
        self.bus.write_byte_data(self.SLAVE_ADDRESS, self.CTRL_REG1, self.data)
        self.bus.write_byte_data(self.SLAVE_ADDRESS, self.CTRL_REG4, self.BDU)
        self.set_range('RANGE_2G')
    
    def set_range(self, range):         
        data = self.bus.read_i2c_block_data(self.SLAVE_ADDRESS, self.CTRL_REG4)
        if range == 'RANGE_4G':
            data = self.FS0
        elif range == 'RANGE_8G':
            data |= self.FS0 |self.FS0
        
        self.scale_factor = self.ACC_RANGES[range]
        if range !=  'RANGE_2G': 
            self.bus.write_byte_data(self.SLAVE_ADDRESS, self.CTRL_REG4, self.data)
        

    # def acc_sleep(toSleep):
    #     data = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.CTRL_REG1)
    #     if toSleep:
    #         data &= ~self.PM0 & 0xFF
    #     else:
    #         data |= self.PM0
    #     self.bus.write_byte(self.CTRL_REG1, data)

    def readXYZ(self):
        status = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.STATUS_REG)
        if not status & self.ZYXDA:
            return
        data = [0, 0, 0, 0, 0, 0]
        
        data = self.bus.read_i2c_block_data(self.SLAVE_ADDRESS,  self.OUT_X_L | 0x80)
        data = data[:6]

        bs = bytes(data)
        # < -little-endian, h - short type: 2 bytes with sign
        x, y, z = struct.unpack("<hhh", bs)
              
        # x = data[0] | data[1] << 8
        # y = data[2] | data[3] << 8
        # z = data[4] | data[5] << 8
        
        # data = [x, y, z]
        # for i, d in enumerate(data):
        #     if d > 2 ** 15:
        #         data[i] -= 2 ** 16
        # x, y, z = data[0], data[1], data[2]
        return x, y, z


    def readX(self):
        status = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.STATUS_REG)
        if not status & self.XDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.OUT_X_L)
        data[1] = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.OUT_X_H)
        
        bs = bytes(data)
        x = struct.unpack("<h", bs)
        return x

    def readY(self):
        status = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.STATUS_REG)
        if not status & self.YDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.OUT_Y_H)
        data[1] = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.OUT_Y_H)

        bs = bytes(data)
        y = struct.unpack("<h", bs)
        return y
    
    def readZ(self):
        status = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.STATUS_REG)
        if not status & self.ZDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.OUT_Z_L)
        data[1] = self.bus.read_byte_data(self.SLAVE_ADDRESS, self.OUT_Z_H)
    
        bs = bytes(data)
        y = struct.unpack("<h", bs)
        return z

    def read_acc_gXYZ(self):
        try:
            #x, y, z = self.readXYZ()
            x = self.readX()
            y = self.readY()
            z = self.readZ()

            self.ax = x * self.scale_factor
            self.ay = y * self.scale_factor
            self.az = z * self.scale_factor
        except Exception:
            return

    



    def read_acc_aXYZ(self):
        self.read_acc_gXYZ()
        self.ax *= self.GRAVITY_EARTH 
        self.ay *= self.GRAVITY_EARTH
        self.az *= self.GRAVITY_EARTH


 


acc = LIS331DLH()
acc.read_acc_aXYZ()
print(acc.ax, acc.ay, acc.az)