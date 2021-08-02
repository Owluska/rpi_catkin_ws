#! /usr/bin/env python3
import time
from drivers.mpu9250_lib import mpu9250

path = 'src/rpicar/src/scripts/data/'
name = 'imu_offsets.txt'
cf = open(path+name, 'w')

def gyro_calibration():
    mpu_array = [] #imu array for gyro vals
cf.close()