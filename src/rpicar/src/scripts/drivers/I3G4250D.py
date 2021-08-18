#! /usr/bin/env python3
import smbus
from time import sleep
import struct

class I3G4250D:
    ADDRESS = 0x68
    #regs
    REF_P_XL = 0x08
    REF_P_L = 0x09
    REF_P_H = 0x0A
    
    WHO_AM_I = 0x0F
    


    def __init__(self):
        class RG:
            def __init__(self, name, dps, sens, bits):
                self.name = name
                self.sensetivity = sens * 1e-3
                self.scale_factor = dps * 2/ 2 ** 16
                self.bits = bits
        class DR:
            def __init__(self, name, bits, freq):
                self.name = name
                self.freq = freq
                self.bits = bits

        
        self.bus = smbus.SMBus(1) # start comm with i2c bus
        


        self.scale = 245 * 2 / 2 ** 16
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0

    def start(self, bandwidth, data_rate):
        data = self.XEN | self.YEN | self.ZEN | self.PD
        data |= bandwidth.bits
        data |= data_rate.bits
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG1, data)

        
    def hp_filter_set(self, mode, cutoff, enable = True):
        data = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG5)
        if enable:
            data |= self.HPen
            self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG5, data)
        else:
            data &= ~ self.HPen & 0xFF
        
        data = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG2)
        data |= mode | cutoff
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG1, data)

    def set_range(self, data_range):
        data = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG4)
        data |= data_range.bits
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG4, data)
        
        self.scale = data_range.scale_factor
        self.sensetivity = data_range.sensetivity

    def readXYZ(self):
        status = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG)
        if not status & self.ZYXDA:
            return
        data = [0, 0, 0, 0, 0, 0]
        
        data = self.bus.read_i2c_block_data(self.ADDRESS,  self.OUT_X_L | 0x80)
        data = data[:6]

        bs = bytes(data)
        BLE = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG4) & self.BLE
        if BLE:
            read_str = ">hhh"
        else:
            # < -little-endian, h - short type: 2 bytes with sign
            read_str = "<hhh"

        self.rx, self.ry, self.rz = struct.unpack(read_str, bs)
              
    def readX(self):
        status = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG)
        if not status & self.XDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.ADDRESS, self.OUT_X_L)
        data[1] = self.bus.read_byte_data(self.ADDRESS, self.OUT_X_H)
        
        bs = bytes(data)      
        self.rx = struct.unpack("<h", bs)[0]

    def readY(self):
        status = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG)
        if not status & self.YDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Y_H)
        data[1] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Y_H)

        bs = bytes(data)   
        self.ry = struct.unpack("<h", bs)[0]
    
    def readZ(self):
        status = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG)
        if not status & self.ZDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Z_L)
        data[1] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Z_H)
    
        bs = bytes(data)     
        self.rz = struct.unpack("<h", bs)[0]

    def read_mbarXYZ(self):
        self.readXYZ()
        self.x  = self.rx * self.scale
        self.y  = self.ry * self.scale 
        self.z  = self.rz * self.scale  


gyro = I3G4250D()
gyro.read_mbarXYZ()
# print(gyro.sensetivity)
print(gyro.x, gyro.y, gyro.z)
