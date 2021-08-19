#! /usr/bin/env python3
import smbus
from time import sleep
import struct

class LIS3MDL:
    ADDRESS =  0x1C
    #REGISTER MAP
    WHO_AM_I = 0x0F
    
    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    CTRL_REG3 = 0x22
    CTRL_REG4 = 0x23
    CTRL_REG5 = 0x24
    
    STATUS_REG = 0x27
    
    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_L = 0x2A
    OUT_Y_H = 0x2B
    OUT_Z_L = 0x2C
    OUT_Z_H = 0x2D

    TEMP_OUT_L = 0x2E
    TEMP_OUT_H = 0x2F
    #BITs
    ST = 0x01
    FAST_ODR = 0x02
    DO0 = 0x04
    DO1 = 0x08
    DO2 = 0x10
    OM0 = 0x20
    OM1 = 0x40
    TEMP_EN = 0x80
    
    SOFT_RST = 0x04
    REBOOT = 0x08
    FS0 = 0x20
    FS1 = 0x30

    MD0 = 0x01
    MD1 = 0x02
    LP = 0x40

    BLE = 0x02
    OMZ0 = 0x04
    OMZ1 = 0x08


    FAST_READ = 0x80
    BDU = 0x40

    XDA = 0x01
    YDA = 0x02
    ZDA = 0x04
    ZYXDA = 0x08
    XOR = 0x10
    YOR = 0x20
    ZOR = 0x40
    ZYXOR = 0x80
    
    def __init__(self):
        class ODR:
            def __init__(self, name, output_data_rate, bits=0x00):
                self.name = name
                self.ODR = output_data_rate
                self.bits = bits
        class DR:
            def __init__(self, name, data_range, bits=0x00):
                self.name = name
                self.dr = data_range
                self.scale = data_range * 2 / 2 ** 16
                self.bits = bits
        
        class OMZ:
            def __init__(self, operating_mode, bits=0x00):
                self.name = operating_mode
                self.bits = bits
        
        self.bus = smbus.SMBus(1)

        self.ODR_1000 = ODR("1000Hz", 1000, self.FAST_ODR)
        self.ODR_560 = ODR("560Hz", 560, self.FAST_ODR|self.OM0)
        self.ODR_300 = ODR("300Hz", 300, self.FAST_ODR|self.OM1)
        self.ODR_155 = ODR("155Hz", 155, self.FAST_ODR|self.OM1|self.OM0)
        self.ODR_80 = ODR("80Hz", 80, self.DO2|self.DO1|self.DO0)
        self.ODR_40 = ODR("40Hz", 40, self.DO2|self.DO1)
        self.ODR_20 = ODR("20Hz", 20, self.DO2|self.DO0)
        self.ODR_10 = ODR("10Hz", 10, self.DO2)
        self.ODR_5 = ODR("5Hz", 5, self.DO1|self.DO0)
        self.ODR_2_5 = ODR("2.5Hz", 2.5, self.DO1)
        self.ODR_1_25 = ODR("1.25Hz", 1.25, self.DO0)
        self.ODR_0_625 = ODR("0.625Hz", 0.625)

        self.OMZ_LP = OMZ("low_power")
        self.OMZ_MP = OMZ("medium perfomance mode", self.OMZ0)
        self.OMZ_HP = OMZ("high perfomance mode", self.OMZ1)
        self.OMZ_UHP = OMZ("ultra-high perfomance mode", self.OMZ1|self.OMZ0)
        
        self.DR_4GAUSS = DR("4GAUSS", 4)
        self.DR_8GAUSS = DR("8GAUSS", 8, self.FS0)
        self.DR_12GAUSS = DR("12GAUSS", 12, self.FS1)
        self.DR_16GAUSS = DR("16GAUSS", 16, self.FS1|self.FS0)
        
        self.data_rate = self.ODR_1000
        self.setup(self.data_rate)
        sleep(.5)
        self.scale = 2 * 4 / 2 ** 16
        self.set_range(self.DR_4GAUSS)
        self.set_Z_axis_operating_mode(self.OMZ_UHP)

        # print(self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG1))
        # print(self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG2))
        # print(self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG3))
        # print(self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG4))

        self.x, self.y, self.z = 0,0,0
        self.rx, self.ry, self.rz = 0,0,0
        self.dt = 1/self.data_rate.ODR


    def setup(self, odr, block_reading = True, temp_sensor = True, big_endian = False, fast_read = False):
        # data = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG1)
        # data &= self.TEMP_EN
        if temp_sensor:
            data = self.TEMP_EN
        data |= odr.bits
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG1, data)

        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG3, 0x00)
        
        if block_reading:
            self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG5, self.BDU)
        if big_endian:
            self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG4, self.BLE)
        if fast_read:
            self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG5, self.FAST_READ)
    
    def set_range(self, data_range):
        data = data_range.bits
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG2, data)
        self.scale = data_range.scale

    def soft_reset(self):
        data = self.SOFT_RST
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG2, data)
    
    def reboot(self):
        data = self.REBOOT
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG2, data)
    
    def set_single_conversion_mode(self):
        data = self.MD0
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG3, data)

    def set_Z_axis_operating_mode(self, operating_mode):
        data = operating_mode.bits
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG4, data)
    
    def readXYZ(self):
        BDU = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG5) & self.BDU
        if not BDU:
            return
        
        ZYXDA = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG) & self.ZYXDA
        if not ZYXDA:
            #print(self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG))
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
        XDA = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG) & self.XDA
        if not XDA:
            print(self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG))
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.ADDRESS, self.OUT_X_L)
        data[1] = self.bus.read_byte_data(self.ADDRESS, self.OUT_X_H)
        
        bs = bytes(data)      
        self.rx = struct.unpack("<h", bs)[0]

    def readY(self):
        YDA = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG) & self.YDA
        if not YDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Y_H)
        data[1] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Y_H)

        bs = bytes(data)   
        self.ry = struct.unpack("<h", bs)[0]
    
    def readZ(self):
        ZDA = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG) & self.ZDA
        if not ZDA:
            return
        data = [0, 0]
        data[0] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Z_L)
        data[1] = self.bus.read_byte_data(self.ADDRESS, self.OUT_Z_H)
    
        bs = bytes(data)     
        self.rz = struct.unpack("<h", bs)[0]

    def read_gaussXYZ(self):
        self.readXYZ()
        self.x  = self.rx * self.scale
        self.y  = self.ry * self.scale 
        self.z  = self.rz * self.scale 
    
    def read_gaussXY(self):
        self.readX()
        self.readY()
        self.x  = self.rx * self.scale
        self.y  = self.ry * self.scale

    def read_Temperature(self):
        data = self.bus.read_i2c_block_data(self.ADDRESS, self.TEMP_OUT_L | 0x80, 2)
        LSB = 8 
        bs = bytes(data)
   
        temp_raw = struct.unpack("<h", bs)[0]
        #print(temp_raw * LSB)



# mag = LIS3MDL()
# mag.read_Temperature()
 
# mag.read_gaussXYZ()
# print(mag.x, mag.y, mag.z)
