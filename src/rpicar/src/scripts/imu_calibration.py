#! /usr/bin/env python3
import time
import json
import numpy as np
from scipy.optimize import curve_fit
from drivers.mpu9250_lib import mpu9250



class mpu_calibration:
    path = 'src/rpicar/src/scripts/data/'   
    def __init__(self, name = 'imu_offsets.txt', open_option = 'w'):
        self.name = name    
        self.cf = open(self.path+self.name, open_option)

        self.mpu = mpu9250()
        self.mpu.MPU6050_start()
        self.mpu.AK8963_start()

        self.data_size = 500

        #self.raw_data = np.zeros((self.data_size,9))
        self.gyro_offsets = np.zeros((1,3))
        self.acc_offsets = np.zeros((2,3))
        self.mag_coeffs = np.zeros((2,3))
        
        self.keys = ['gx_off', 'gy_off', 'gz_off', 
                     'ax_slp', 'ay_slp', 'az_slp',
                     'ax_off', 'ay_off', 'az_off',
                     'mx_slp', 'my_slp', 'mz_slp',
                     'mx_off', 'my_off', 'mz_off']
        self.json_str = ""
        self.dict = {}

    def get_gyro(self, func):
        _,_,_,wx,wy,wz = self.mpu.mpu6050_conv() # read and convert gyro data
        return wx,wy,wz

    def get_acc(self):
        ax,ay,az,_,_,_ = self.mpu.mpu6050_conv() # read and convert acc data
        return ax,ay,az

    def acc_fit_finc(self, x, k, b):
        y = k * x + b
        #print(y.shape, x.shape, k, b)
        return y

    def gyro_calibration(self, gyro_func):
        attempts = 0
        gyro_data = np.zeros((self.data_size,3))
        for i in range(self.data_size):
            try:
                gyro = np.array(gyro_func)
                self.gyro_data[i] = gyro
                attempts = 0
            except Exception:
                if attempts > 10:
                    print("Sensor is probably not connected! Returning..")
                    return -1
                attempts += 1
                continue 
        
        self.gyro_offsets = np.mean(gyro_data, axis = 0)
        return 0

    def acc_calibration(self, acc_func):
        #for each axis mowing IMU in three directons
        # 'upward', 'downward', 'perpendicular to gravity' and then 
        # trying to find fitting coefficients as -1, 1, 0 
        axes = ['x', 'y', 'z']
        indices = [2, 1, 0]
        directions = ['upward', 'downward', 'perpendicular to gravity']
        acc_data = np.zeros((self.data_size,3))
        for i, a in enumerate(axes):
            x_data = np.zeros((self.data_size,3))
            for j, d in enumerate(directions):
                template = "Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the - "
                template = "{}{}-axis pointed {}".format(template, a, d)
                
                input(template)

                for k in range(self.data_size):
                    try:
                        acc_data[k] = np.array(acc_func)
                        #self.raw_data[k, 3:6] = acc
                        #print(acc)
                        
                    except Exception:
                        continue          
                #add data array from axis pointing to the gravity, see axes variable
                x_data[:, j] = acc_data[:, indices[i]]

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

    def outlier_removal(self, data, stdev_mult = 5.0):
        #print(data.shape)
        diff = np.diff(data, axis=0)

        edge = np.abs(np.mean(diff, axis=0)) + stdev_mult * np.std(diff)
        #print(edge, diff)
        diff = np.abs(diff)      
        outliers = np.where(diff > edge)
 

        if len(outliers) != 0:
            data[outliers] = np.nan
        return data

    def mag_calibration(self, mag_func):
        # [[x,y], [y,z], [x,z]]
        mag_cal_axes = ['z','y','x']
        mag_calib_data = []
        for i, axis in enumerate(mag_cal_axes):
            template = "Press Enter and Start Rotating the IMU Around the {}-axis".format(axis)
            input(template)
            print("\t When Finished, Press CTRL+C")
            mag_data = np.zeros((1,3))

            while True:
                try:
                    mags = np.array(mag_func).reshape((1,3)) # read and convert AK8963 magnetometer data
                    mag_data = np.append(mag_data, mags, axis = 0)
                except (Exception, KeyboardInterrupt) as e:  
                    if type(e).__name__ == 'KeyboardInterrupt':
                        break
                    else:
                        continue
            mag_data = mag_data[20:] #throw away first 20 points
            mag_calib_data.append(mag_data)
        
        mag_calib_data = np.array(mag_calib_data)

        # indices axes for offsets and slope for each axis
        #based on this paper with DOI:
        #10.1109/ICCE.2019.8661986
        slop_offs_indicies = [[0,1],[1,0],[2,2]]
        for i, vec in enumerate(mag_calib_data):
            m1, m2 = vec[:, slop_offs_indicies[i][0]], vec[:, slop_offs_indicies[i][1]]
            #fills np.nan all outlier values
            m1 = self.outlier_removal(m1)
            m2 = self.outlier_removal(m2)
       
            slope = np.nanmax(m2) - np.nanmin(m2)/np.nanmax(m1) - np.nanmin(m1)
            offset = (np.nanmax(m1) + np.nanmin(m1))/2.0 - np.nanmax(m1) * slope         

            self.mag_coeffs[0, i] = offset
            self.mag_coeffs[1, i] = slope
               

            

    def data_to_json(self, keys, data):    
        # l = len(keys)
        # for i, k, go in zip(range(l), keys, data):
        #     self.json_str += "\"{:s}\":{}".format(k, go)
        #     if i < l - 1:
        #         s += ", "
        #         l = len(keys)
        for k, go in zip(keys, data):
            self.json_str += "\"{:s}\":\"{}\", ".format(k, go)
        return self.json_str
    
    def json_to_dict(self):
        self.dict = json.loads(self.json_str)
    
    def write_file(self):
        self.data_to_json(self.keys[:3], self.gyro_offsets)
        
        self.data_to_json(self.keys[3:6], self.acc_offsets[0])
        self.data_to_json(self.keys[6:9], self.acc_offsets[1])
        
        self.data_to_json(self.keys[9:12], self.mag_coeffs[0])
        self.data_to_json(self.keys[12:15], self.mag_coeffs[1])

        #remove last comma and space
        self.json_str = self.json_str[:-2]
        #add curly brackets
        self.json_str = "{" + self.json_str + "}"
        self.cf.write(self.json_str)
        self.cf.write('\n')
        # save data to dictonary, usefull for debugging 
        self.json_to_dict()
    
    def calibrate_all(self, gyro_func, acc_func, mag_func):
        res = cal.gyro_calibration(gyro_func)
        if res == -1:
            self.cf.close()
        else:
            cal.acc_calibration(acc_func)
            cal.mag_calibration(mag_func)
            cal.write_file()

    def close_file(self):
        self.cf.close()

from drivers.multiplexer import PCA9547
from drivers.LIS331DLH import LIS331DLH

acc = LIS331DLH()
acc.hp_filter_setup(acc.hp_freq.bits, acc.hp_modes['reference'])

def get_acc_data(acc):   
    acc.readXYZ()
    acc.raw_to_ms()
    return acc.x, acc.y, acc.z 

pca = PCA9547()

cal = mpu_calibration(open_option='a')
cal.calibrate_all(get_acc_data)
print(cal.dict)

