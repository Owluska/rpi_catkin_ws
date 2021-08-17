#! /usr/bin/env python3
import smbus
from time import sleep
import struct

class I3G4250D:
    ADDRESS = 0x68
    #regs
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
    #bits
    XEN = 0x01
    YEN = 0x02
    ZEN = 0x04
    PD  = 0x08
    BW0 = 0x10
    BW1 = 0x20
    DR0 = 0x40
    DR1 = 0x80

    HPCF0 = 0x01
    HPCF1 = 0x02
    HPCF2 = 0x04
    HPCF3 = 0x08
    HPM0 = 0x10
    HPM1 = 0x20

    STO = 0x02
    ST1 = 0x04
    FS0 = 0x10
    FS1 = 0x20
    BLE = 0x40

    HPen = 0x10
    
    XDA = 0x01
    YDA = 0x02
    ZDA = 0x04
    ZYXDA = 0x08

    hpf_bs = [      0x00,       HPCF0,             HPCF1, HPCF1|HPCF0,       HPCF2,
             HPCF2|HPCF0, HPCF2|HPCF1, HPCF2|HPCF1|HPCF0,       HPCF3, HPCF3|HPCF0]

    hpf_100Hz = ['8',   '4',  '2', '1', '0.5', '0.2', '0.1', '0.05', '0.02', '0.01']
    hpf_200Hz = ['15',  '8',  '4', '2',   '1', '0.5', '0.2',  '0.1', '0.05', '0.02']
    hpf_400Hz = ['30', '15',  '8', '4',   '2',   '1', '0.5',  '0.2',  '0.1', '0.05']
    hpf_800Hz = ['56', '30', '15', '8',   '4',   '2',   '1',  '0.5',  '0.2',  '0.1']

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
        class BW:
            def __init__(self, name, bits, dr_freq):
                self.name = name
                self.bits = bits
                self.freq = dr_freq
        class HP_modes:
            def __init__(self, high_bit, low_bit):
                self.normal = high_bit
                self.reference = low_bit
                self.autoreset = high_bit | low_bit 
        
        self.bus = smbus.SMBus(1) # start comm with i2c bus
        self.RG_245 = RG('245dps', 245, 8.75, 0x00)
        self.RG_500 = RG('500dps', 500, 17.5, self.FS0)
        self.RG_2000 = RG('2000dps', 2000, 70.0, self.FS1)

        self.DR_100 = DR('DR_100', 0x00, 100)
        self.DR_200 = DR('DR_200', self.DR0, 200)
        self.DR_400 = DR('DR_400', self.DR1, 400)
        self.DR_800 = DR('DR_800', self.DR1 | self.DR0, 800)


        self.BW_110 = BW('BW_110', self.BW1|self.BW0, self.DR_800.freq)

        self.HP_freqs_DR100 = {f:b for f,b in zip(self.hpf_bs, self.hpf_100Hz)}
        self.HP_freqs_DR200 = {f:b for f,b in zip(self.hpf_bs, self.hpf_200Hz)}
        self.HP_freqs_DR400 = {f:b for f,b in zip(self.hpf_bs, self.hpf_400Hz)}
        self.HP_freqs_DR800 = {f:b for f,b in zip(self.hpf_bs, self.hpf_800Hz)}
        
        self.hp_mode = HP_modes(self.HPM1, self.HPM0)

        self.scale = 245 * 2 / 2 ** 16
        self.sensetivity = 8.75 * 1e-3
        self.start(self.BW_110, self.DR_800)       
        self.set_range(self.RG_245)
        
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

    def read_degXYZ(self):
        self.readXYZ()
        self.x  = self.rx * self.sensetivity
        self.y  = self.ry * self.sensetivity 
        self.z  = self.rz * self.sensetivity  


gyro = I3G4250D()
gyro.read_degXYZ()
# print(gyro.sensetivity)
print(gyro.x, gyro.y, gyro.z)


