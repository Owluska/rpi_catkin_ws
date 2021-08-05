#! /usr/bin/env python3
import time
import json
import numpy as np
from scipy.optimize import curve_fit
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
        self.gyro_offsets = np.zeros((1,3))
        self.acc_offsets = np.zeros((2,3))
        
        self.keys = ['gx_off', 'gy_off', 'gz_off', 
                     'ax_slp', 'ay_slp', 'az_slp',
                     'ax_off', 'ay_off', 'az_off']
        self.json_str = ""
        self.dict = {}

    def get_gyro(self):
        _,_,_,wx,wy,wz = self.mpu.mpu6050_conv() # read and convert gyro data
        return wx,wy,wz

    def get_acc(self):
        ax,ay,az,_,_,_ = self.mpu.mpu6050_conv() # read and convert acc data
        return ax,ay,az

    def acc_fit_finc(self, x, k, b):
        y = k * x + b
        #print(y.shape, x.shape, k, b)
        return y

    def gyro_calibration(self):
        for i in range(self.data_size):
            try:
                gyro = np.array(self.get_gyro())
                self.raw_data[i, :3] = gyro
            except Exception:
                continue 
        
        self.gyro_offsets = np.mean(self.raw_data[:, :3], axis = 0)

    def acc_calibration(self):
        #for each axis mowing IMU in three directons
        # 'upward', 'downward', 'perpendicular to gravity' and then 
        # trying to find fitting coefficients as -1, 1, 0 
        axes = ['x', 'y', 'z']
        indices = [2, 1, 0]
        directions = ['upward', 'downward', 'perpendicular to gravity']

        for i, a in enumerate(axes):
            x_data = np.zeros((self.data_size,3))
            for j, d in enumerate(directions):
                template = "Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the - "
                template = "{}{}-axis pointed {}".format(template, a, d)
                
                input(template)

                for k in range(self.data_size):
                    try:
                        acc = np.array(self.get_acc())
                        self.raw_data[k, 3:6] = acc
                        #print(acc)
                        
                    except Exception:
                        continue 
                
                    #add data array from axis pointing to the gravity, see 
                    #print(i, j, len(indices),  x_data.shape, self.raw_data.shape)
                
                x_data[:, j] = self.raw_data[:, 3 + indices[i]]

            #using calibrations (+1g, -1g, 0g) for linear fit
            ones = np.ones(self.data_size, dtype = 'float')
            y_data = np.vstack((-1 * ones, 1 * ones, 0 * ones)).T
            y = y_data.ravel()     
            x = x_data.ravel()
            #print(y.shape, x.shape)

            coeffs, _ = curve_fit(self.acc_fit_finc, x, y)
            #print(coeffs)
            self.acc_offsets[0, indices[i]] = coeffs[0]
            self.acc_offsets[1, indices[i]] = coeffs[1]

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
        self.data_to_json(self.keys[:3], self.gyro_offsets)
        self.data_to_json(self.keys[3:6], self.acc_offsets[0])
        self.data_to_json(self.keys[6:9], self.acc_offsets[1])
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

cal.gyro_calibration()
cal.acc_calibration()
cal.write_file()
print(cal.dict)

