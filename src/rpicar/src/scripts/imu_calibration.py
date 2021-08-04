#! /usr/bin/env python3
import time
import json
import numpy as np

from drivers.mpu9250_lib import mpu9250



class mpu_calibration:
    path = 'src/rpicar/src/scripts/data/'
    name = 'imu_offsets.txt'    
    cf = open(path+name, 'w')
    
    def __init__(self):
        self.mpu = mpu9250()
        self.mpu.MPU6050_start()
        self.mpu.AK8963_start()

        self.data_size = 500

        self.raw_data = np.zeros((self.data_size,9))
        self.gyro_offsets = np.zeros((self.data_size,3))
        
        self.keys = ['gx_off', 'gy_off', 'gz_off']
        self.json_str = ""
        self.dict = {}

    def get_gyro(self):
        _,_,_,wx,wy,wz = self.mpu.mpu6050_conv() # read and convert gyro data
        return wx,wy,wz

    def get_acc(self):
        ax,ay,az,_,_,_ = self.mpu.mpu6050_conv() # read and convert acc data
        return ax,ay,az

    def acc_fit_finc(self, x, k, b):
        return k*x + b

    def gyro_calibration(self):
        for i in range(self.data_size):
            try:
                gyro = np.array(self.get_gyro())
                self.raw_data[i, :3] = gyro
            except Exception:
                continue 
        
        self.gyro_offsets = np.mean(self.raw_data[:, :3], axis = 0)

    def acc_calibration(self)

    def data_to_json(self, keys, data):    
        # l = len(keys)
        # for i, k, go in zip(range(l), keys, data):
        #     self.json_str += "\"{:s}\":{}".format(k, go)
        #     if i < l - 1:
        #         s += ", "
        #         l = len(keys)
        for k, go in zip(keys, data):
            self.json_str += "\"{:s}\":{}, ".format(k, go)

        return self.json_str
    
    def json_to_dict(self):
        self.dict = json.loads(self.json_str)
    
    def write_file(self):
        self.data_to_json(self.keys, self.gyro_offsets)
        #remove last comma and space
        self.json_str = self.json_str[:-2]
        #add curly brackets
        self.json_str = "{" + self.json_str + "}"
        self.cf.write(self.json_str)
        # save data to dictonary, usefull for debugging 
        self.json_to_dict()

    def close_file(self):
        self.cf.close()

cal = mpu_calibration()

# cal.gyro_calibration()
# cal.write_file()
# print(cal.dict)

