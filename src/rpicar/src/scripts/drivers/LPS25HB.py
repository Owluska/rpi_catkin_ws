## %%
#! /usr/bin/env python3
import smbus
from time import sleep
import struct

class LPS25HB:
    ADDRESS = 0x5C
    #regs
    REF_P_XL = 0x08
    REF_P_L = 0x09
    REF_P_H = 0x0A
    
    WHO_AM_I = 0x0F
    
    RES_CONF = 0x10
    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    CTRL_REG3 = 0x22
    CTRL_REG4 = 0x23

    STATUS_REG = 0x27

    PRESS_OUT_XL = 0x28
    PRESS_OUT_L = 0x29
    PRESS_OUT_H = 0x2A
    
    TEMP_OUT_L = 0x2B
    TEMP_OUT_H = 0x2C

    THS_P_L = 0x30
    THS_H_L = 0x31

    RPDS_L = 0x39
    RPDS_H = 0x3A
    #bits
    AVGP0 = 0x01
    AVGP1 = 0x02
    AVGT0 = 0x04
    AVGT1 = 0x08
 
    RESET_AZ = 0x02
    BDU  = 0x04
    ODR0 = 0x10
    ODR1 = 0x20
    ODR2 = 0x40
    PD = 0x80

    ONE_SHOT  = 0x01
    AUTO_ZERO = 0x02
    SWRESET   = 0x04
    I2C_DIS   = 0x08
    FIFO_MEAN_DEC   = 0x10
    STOP_ON_FTH = 0x20
    FIFO_EN = 0x40
    BOOT = 0x80

    INT_S1 = 0x01
    INT_S2 = 0x04
    PP_OD  = 0x40
    INT_H_L  = 0x80

    DRDY = 0x01
    F_OVR = 0x02
    F_FTH = 0x04
    F_EMPTY = 0x10

    T_DA = 0x01
    P_DA = 0x02
    T_OR = 0x10
    P_OR = 0x20

    mB2mmHg = 0.750061682
    T_SCALE = 1/480
    T_OFFSET = 42.5
    
    def __init__(self, measure_delta = False):      
        class DR:
            def __init__(self, name, data_rate, bits = 0x00):
                self.name = name
                self.data_rate = data_rate
                self.bits = bits
        class AVG:
            def __init__(self, name, avg, bits = 0x00):
                self.name = name
                self.avg = avg
                self.bits = bits           
        
        self.bus = smbus.SMBus(1) # start comm with i2c bus       
        
        self.DR_oneshot = DR('One shot', 0)
        self.DR_1 = DR('1 Hz', 1, self.ODR0)
        self.DR_7 = DR('7 Hz', 7, self.ODR1)
        self.DR_12_5 = DR('12.5 Hz', 12.5, self.ODR0 | self.ODR1)
        self.DR_25 = DR('25 Hz', 25, self.ODR2)

        self.AVGT_8 = AVG('AVGT_8', 8)
        self.AVGT_16 = AVG('AVGT_16', 16, self.AVGT0)
        self.AVGT_32 = AVG('AVGT_32', 32, self.AVGT1)
        self.AVGT_64 = AVG('AVGT_64', 64, self.AVGT1 | self.AVGT0)

        self.AVGP_8 = AVG('AVGP_8', 8)
        self.AVGP_32 = AVG('AVGP_32', 32, self.AVGP0)
        self.AVGP_128 = AVG('AVGT_128', 128, self.AVGP1)
        self.AVGP_512 = AVG('AVGT_512', 512, self.AVGP1 | self.AVGP0)
        
        self.scale = 1/ 2 ** 12
        # self.avgt = 8
        # self.avgp = 8
        self.mDelta = measure_delta
        self.data_rate = self.DR_25
        self.start(self.AVGT_16, self.AVGP_512, self.data_rate, self.mDelta)  
        
        self.pressure = 0.0
        self.temp = 0.0
        self.dt = 1 / self.data_rate.data_rate

    def start(self, temp_averaging, pressure_averaging, data_rate, measure_delta):
        self.setup_RES_CONF(temp_averaging, pressure_averaging)
        self.setup_CTRL_REG1(data_rate, reset_autozero = not measure_delta)
        self.setup_CTRL_REG2(auto_zero = measure_delta)

    
    def setup_RES_CONF(self, temp_averaging, pressure_averaging):
        data = 0x00
        
        data |= temp_averaging.bits
        
        data |= pressure_averaging.bits
        
        self.bus.write_byte_data(self.ADDRESS, self.RES_CONF, data)
        
        # self.avgt = temp_averaging.avg
        # self.avgp = pressure_averaging.avg

    def setup_CTRL_REG1(self, data_rate, power_down_mode = False, block_data_reading = True, reset_autozero = False):
        data = data_rate.bits
        if not power_down_mode:
            data |= self.PD

        if block_data_reading:
            data |= self.BDU

        if reset_autozero:
            data |= self.AUTO_ZERO
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG1, data)
        

        
    def setup_CTRL_REG2(self, auto_zero = False, soft_reset = False, fifo_mean_dec = False,
                             stop_on_fifo_thrsh = False, fifo_enable = False, boot = False):
        data = 0x00
        if auto_zero:
            data |= self.AUTO_ZERO
        if soft_reset:
            data |= self.SWRESET
        if fifo_mean_dec:
            data |= self.FIFO_MEAN_DEC
        if stop_on_fifo_thrsh:
            data |= self.STOP_ON_FTH
        if fifo_enable:
            data |= self.FIFO_EN
        if boot:
            data |= self.BOOT
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG2, data)


    def readPressure(self):
        P_DA = self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG) & self.P_DA
        #print(self.bus.read_byte_data(self.ADDRESS, self.STATUS_REG))
        if not P_DA:
            return
        
        data = self.bus.read_i2c_block_data(self.ADDRESS,  self.PRESS_OUT_XL | 0x80, 3) 
        #print(data)
        negative = data[2] & 0x80
        #data[2] &= 0x7F
        #print(negative)      
        if negative:
            data[2] -= 255
        
        p = data[2] << 16 | data[1] << 8 | data[0]
        self.pressure = p * self.scale
        if negative:
            self.pressure *= -1

    def set_delta_measurment(self, on = True):      
        if on:
            data = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG2)
            data |= self.AUTO_ZERO
            self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG2, data)
        else:               
            data = self.bus.read_byte_data(self.ADDRESS, self.CTRL_REG1)
            data |= self.RESET_AZ
            self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG1, data)
    
    def readTemperature(self):
        data = self.bus.read_i2c_block_data(self.ADDRESS,  self.TEMP_OUT_L | 0x80, 2)
        bs = bytes(data)
        temp_raw = struct.unpack(">h", bs)[0]
        self.temp = self.T_OFFSET + temp_raw * self.T_SCALE
        print(self.temp)

        
               


# bar = LPS25HB()
# for i in range(20):
#     #print(bar.mDelta)
#     if i == 10:
#         bar.set_delta_measurment(on=True)
#         sleep(.5)
#     bar.readPressure()
#     template = "Pressure {:.4f}mBar, {:.4f}mmHg, dt:{:.2f}s".format(bar.pressure, bar.pressure * bar.mB2mmHg, bar.dt)
#     print(template)
#     sleep(bar.dt)

# bar = LPS25HB()
# bar.readTemperature()
