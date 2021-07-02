#! /usr/bin/env python3
import smbus
import time
import sys

class PCA9547():
    ADDRESS      = 0x70
    ENABLE_MASK  = 0x08
    DEFAULT_CHANNEL = 0
    COUNT_CHANNEL   = 8

    def __init__(self):
        self.bus = smbus.SMBus(1) # start comm with i2c bus
        self.status = 0x00
        time.sleep(0.1)
        self.begin()
        
    def begin(self):
        self.set_channel(self.DEFAULT_CHANNEL)
    

    def set_channel(self, channel):
        if channel >= self.COUNT_CHANNEL:
            print("Wrong channel number!")
            return
        self.status = self.bus.write_byte(self.ADDRESS, channel | self.ENABLE_MASK)
        time.sleep(0.1)

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


pca = PCA9547()
for i in range(8):
    pca.set_channel(i)
    print("Set channel:{}, status: {}\n".format(i, pca.status))
    time.sleep(1)
    # pca.get_status()
    # print("PCA9547 I2C channel {} status: {}".format(i, pca.status))
    scanner()
    time.sleep(1)


