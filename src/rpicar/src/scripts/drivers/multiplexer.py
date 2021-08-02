#! /usr/bin/env python3
import smbus
import time
import sys

class PCA9547():
    ADDRESS      = 0x74
    ENABLE_MASK  = 0x08
    COUNT_CHANNEL   = 8

    def __init__(self, channel = 0):
        self.bus = smbus.SMBus(1) # start comm with i2c bus
        self.data = 0x00
        time.sleep(0.1)
        self.set_channel(channel)
       
    def set_channel(self, channel):
        if channel >= self.COUNT_CHANNEL:
            print("Wrong channel number!")
            return
        self.bus.write_byte(self.ADDRESS, channel | self.ENABLE_MASK)
        #self.bus.write_byte(self.ADDRESS, channel)
        time.sleep(0.5)
    
    def read_from_channel(self):
        self.data = self.bus.read_byte(self.ADDRESS)
        time.sleep(0.5)

def scanner():
    bus = smbus.SMBus(1)
    print("scanning...")
    for a in range(127):
        # bus.write_byte_data(a)
        # time.sleep(0.1)
        try:
            hex(bus.read_byte(a)) 
            time.sleep(0.1)
            print("Found I2C device on adress {}".format(hex(a)))
        except Exception:
            pass


# pca = PCA9547()
# # pca.set_channel(7)
# for i in range(8):
#     pca.set_channel(i)
#     print("Set channel:{}, data: {}\n".format(i, pca.data))
#     time.sleep(1)
#     data = pca.read_from_channel()
#     print(data)
#     scanner()
#     time.sleep(1)


