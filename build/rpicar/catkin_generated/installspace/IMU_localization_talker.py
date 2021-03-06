#! /usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Imu, MagneticField, Temperature


# from drivers.mpu9250_lib import mpu9250
import math
import ahrs
import json
import numpy as np
from ahrs import Quaternion

from drivers.es_ekf import ekf
from drivers.LIS331DLH import LIS331DLH as Accelerometer
from drivers.I3G4250D import I3G4250D as Gyroscope
from drivers.LIS3MDL import LIS3MDL as Magnetometer
from drivers.LPS25HB import LPS25HB as Barometer
# from filterpy.kalman import ExtendedKalmanFilter

from time import sleep, time

class imu_talker():
    def __init__(self, cf_path):
        # self.offsets = np.array([[0.9964611434970413, -0.1358959565043638],
        #                              [0.998331379179552,0.06877502271143854],
        #                              [-0.9860600182267061,-0.3923817524528095],
        #                               1.7522125244140625,-2.4026947021484375,-0.3843841552734375,
        #                              [145.3125],[35.3759765625],[8.349609375]], dtype = 'object')
        # self.imu = mpu9250()
        self.f = Accelerometer()
        #self.f.hp_filter_setup(self.f.hp_400Hz, self.f.hp_reference)
        self.w = Gyroscope()
        self.m = Magnetometer()
        self.p = Barometer()
        self.imu_coefs = {}
        self.cf = open(cf_path, 'r')
        self.read_IMU_calibration()
        #print(self.imu_coefs)
        
        # self.ax = 0.00
        # self.ay = 0.00
        # self.az = 10.00
        
        # self.gx = 0.00
        # self.gy = 0.00
        # self.gz = 0.00
        
        # self.mx = 0.00
        # self.my = 0.00
        # self.mz = 0.00

        #self.temp = 0.0

        self.dt = 0.00
        self.t = 0.00
        self.av_temperature = 0.0

        self.acc_data = np.array([self.f.x, self.f.y, self.f.z])
        self.gyro_data = np.array([self.w.x, self.w.y, self.w.z]) 
        #self.t_step = self.w.dt + self.f.dt + self.m.dt + self.p.dt
        self.t_step = .02
        self.isMoving = False
        self.isRotating = False 
        
        self.g = 9.84
        self.D2R = math.pi/180
        self.R2D = 180 /math.pi

        self.position = {'x':0.0, 'y':.0, 'z':.0}
        self.orientation = {'roll':0.0, 'pitch':.0, 'yaw':.0}
        
        self.acc_data = np.array([[.0, .0, self.g]])
        self.gyro_data = np.array([[.0, .0, .0]])
        self.mag_data = np.array([[.0,.0,.0]])
        self.quat_data = np.array([[1.0, .0, .0, 0.0]])
        self.use_magnetometer = True
        
        self.learning_gain = 0.001
        if self.use_magnetometer:  
            self.filter = ahrs.filters.Madgwick(acc = self.acc_data, gyr = self.gyro_data, mag = self.mag_data,
                                                q0 = self.quat_data, gain = self.learning_gain, frequency=1/self.t_step)
        else:
            self.filter = ahrs.filters.Madgwick(acc = self.acc_data, gyr = self.gyro_data, q0 = self.quat_data,
                                                gain = self.learning_gain, frequency=1/self.t_step)
    
        
        self.publish = False
        self.I = np.eye(3)

        self.var_w = np.array([0.067, 0.107, 0.029])
        self.var_f = np.array([1.962, 3.31 , 1.603])
        self.var_m = np.array([5.774, 1.119, 1.466])
        
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        self.temp_msg = Temperature()

        self.imu_msg.angular_velocity_covariance = (self.I * self.var_w).flatten()
        self.imu_msg.linear_acceleration_covariance = (self.I * self.var_f).flatten()
        self.mag_msg.magnetic_field_covariance = (self.I * self.var_m).flatten()
        
        
        self.imu_msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.mag_msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.temp_msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link')
        
        self.seq = 0        
           
        
        self.pub_raw = rospy.Publisher("imu/raw", Imu, queue_size = 10)
        self.pub_mag = rospy.Publisher("imu/mag", MagneticField, queue_size = 10)
        self.pub_temp = rospy.Publisher("imu/temp", Temperature, queue_size = 10)

        
        self.print = False
        self.loop_rate= rospy.Rate(0.05)

        self.kf = ekf()
        self.kf.var_f = self.var_f
        self.kf.var_w = self.var_w
        self.kf.var_m = self.var_m
        self.kf.g = np.array([0, 0, -self.g]) 

    # def get_mpu9250_data(self):
    #         self.ax,self.ay,self.az,self.gx,self.gy,self.gz = self.imu.mpu6050_conv() # read and convert mpu6050 data
    #         self.mx,self.my,self.mz = self.imu.AK8963_conv()
    #         return np.array([self.ax,self.ay,self.az,self.gx,self.gy,self.gz,self.mx,self.my,self.mz])
        
    # def calibrated_mpu9250(self):
    #     raw  = self.get_mpu9250_data()
         
    #     mpu_cal = np.zeros_like(raw)
    #     cal_rot_indicies = [[6,7],[7,8],[6,8]] # heading indices
    #     for i in range(3):   
    #         mpu_cal[i] = self.offsets[i][0]*raw[i]+ self.offsets[i][1]
    #     for i in range(3,6):
    #         mpu_cal[i] = raw[i] - self.offsets[i]
    #     for i in range(6,9):
    #         j = i-6
    #         mpu_cal[i] = raw[i] - self.offsets[cal_rot_indicies[j][0]]
    #     self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.mx, self.my, self.mz = mpu_cal
    #     return mpu_cal
    
    def read_IMU_calibration(self):
        data_s = self.cf.read()
        #get index of last measurment 
        start_i = data_s[::-1].find("{") + 1
        #delete other measurments
        data_s = data_s[-start_i:]
        #data to dictonary
        self.imu_coefs =  json.loads(data_s)
    
    def read_IMU_data(self):
        self.f.read_ms2XYZ()
        #sleep(self.f.dt)

        self.w.read_degXYZ()
        self.w.read_Temperature()
        #sleep(self.w.dt)

        self.m.read_gaussXYZ()
        self.m.read_Temperature()
        #sleep(self.m.dt)

        self.p.read_Temperature()
        #sleep(self.p.dt)

        self.av_temperature = self.w.temp + self.p.temp + self.m.temp
        self.av_temperature /= 3

        sleep(self.t_step)

    def read_IMU_calibrated_data(self):
        self.read_IMU_data()
        #print(self.w.x, self.w.y, self.w.z, self.f.x, self.f.y, self.f.z, self.m.x, self.m.y, self.m.z)
        data_array = [self.w.x, self.w.y, self.w.z,
                               self.f.x, self.f.y, self.f.z,
                               self.m.x, self.m.y, self.m.z]
        coefs = self.imu_coefs
        coefs = {k:float(v) for k, v in coefs.items()}
        keys = list(coefs.keys())
        #print(len(keys))
        self.w.x, self.w.y, self.w.z = [d - coefs[k] for d, k in zip(data_array[:3], keys[:3])]
        self.f.x, self.f.y, self.f.z = [d * coefs[ks] -  coefs[koff] 
                                        for d, ks, koff in zip(data_array[3:6], keys[3:6], keys[6:9])]
        # self.f.z += self.f.GRAVITY_EARTH        
        self.m.x, self.m.y, self.m.z = [d * coefs[ks] -  coefs[koff] 
                                        for d, ks, koff in zip(data_array[6:], keys[9:12], keys[12:])]
        #print(self.w.x, self.w.y, self.w.z, self.f.x, self.f.y, self.f.z, self.m.x, self.m.y, self.m.z)
    
    def sensor_data_to_AHRS(self):
        # self.ax *= self.g
        # self.ay *= self.g
        # self.az *= self.g

        self.w.x *= self.D2R
        self.w.y *= self.D2R
        self.w.z *= self.D2R
        #print(self.f.x, self.f.y, self.f.z)
        acc_data = np.array([[self.f.x, self.f.y, self.f.z]])
        self.acc_data = np.append(self.acc_data, acc_data, axis = 0)

        gyro_data = np.array([[self.w.x, self.w.y, self.w.z]])
        self.gyro_data = np.append(self.gyro_data, gyro_data, axis = 0)

        mag_data = np.array([[self.m.x, self.m.y, self.m.z]])
        self.mag_data = np.append(self.mag_data, mag_data, axis = 0)
    
    def read_sensor_data(self):
        try:
            self.read_IMU_calibrated_data()
            self.sensor_data_to_AHRS()
        except Exception as e:
            rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
        

    
    def talker(self, msg):   
        self.read_sensor_data()
        
        #publish imu acceleration and gyro
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.seq = self.seq
        self.imu_msg.angular_velocity.x  = self.w.x
        self.imu_msg.angular_velocity.y = self.w.y
        self.imu_msg.angular_velocity.z = self.w.z
        
        self.imu_msg.linear_acceleration.x = self.f.x
        self.imu_msg.linear_acceleration.y = self.f.y
        self.imu_msg.linear_acceleration.z = self.f.z
        self.pub_raw.publish(self.imu_msg)
        
        #publish magnetometer
        self.mag_msg.header.stamp = rospy.Time.now()
        self.mag_msg.header.seq = self.seq
        self.mag_msg.magnetic_field.x  = self.m.x
        self.mag_msg.magnetic_field.y = self.m.y
        self.mag_msg.magnetic_field.z = self.m.z
        
        self.pub_mag.publish(self.mag_msg)
        
        #publish temperature
        self.temp_msg.header.stamp = rospy.Time.now()
        self.temp_msg.header.seq = self.seq
        self.temp_msg.temperature = self.p.temp
        
        self.pub_temp.publish(self.temp_msg)

    def get_orientation(self):
        self.isRotating = rospy.get_param("rotating_state", default = False)
        self.filter.Dt = self.dt
        if self.use_magnetometer:
            quat_data = self.filter.updateMARG(self.quat_data[-1], gyr = self.gyro_data[-1], acc = self.acc_data[-1], mag = self.mag_data[-1])
      
        else:
            quat_data = self.filter.updateIMU(self.quat_data[-1], gyr = self.gyro_data[-1], acc = self.acc_data[-1])
    
        quat_data = quat_data.reshape((1, 4))
        self.quat_data = np.append(self.quat_data, quat_data, axis = 0)
  
        angles = Quaternion(self.quat_data[-1]).to_angles()
        angles *= self.R2D

        for k, a in zip(self.orientation, angles): 
            self.orientation[k] = float(a)

    def get_position(self):
        self.kf.q_est[-1] = self.quat_data[-1]
        self.isMoving = rospy.get_param("moving_state", default = False)
        #rospy.loginfo("isM: {}".format(self.isMoving))
        if not self.isMoving:
            self.kf.v_est[-1] = np.zeros((1,3))
            self.kf.a[-1] = np.zeros((1,3))
        #k = self.quat_data.shape[0] - 1
        self.kf.update(self.acc_data[-1], self.gyro_data[-1], self.dt)
        coords = self.kf.p_est[-1]
        for k, c in zip(self.position, coords): 
            self.position[k] = float(c)
   

    def start(self):
        tt = time()
        self.print = True
        # print once in a second
        n =  int(1/(self.t_step))
        #print(n, self.t_step)
        while not rospy.is_shutdown():
            self.dt = time() - tt
            tt = time()
            self.t += self.dt

            self.read_sensor_data()
            
            if self.publish:
                self.talker()

            self.get_orientation()
            rospy.set_param("orientation", self.orientation)
            
            #self.get_position()           
            #rospy.set_param("position", self.position)
            
            if self.seq % n == 0 and self.print:
                template = "{:.2f}s {:.2f} {:.2f} {:.2f}".format(self.t, *self.orientation.values())
                #template = "{:.2f}s {:.2f} {:.2f} {:.2f}".format(self.t, self.w.x, self.w.y, self.w.z)
                #template = "{:.2f}s {:.2f} {:.2f} {:.2f}".format(self.t, self.f.x, self.f.y, self.f.z)
                print(template)

            #sleep(self.t_step)
            self.seq += 1
            #self.loop_rate.sleep()

def main(args):
    rospy.init_node('imu_talker', anonymous = True)
    path = '/home/pi/catkin_ws/src/rpicar/src/scripts/data/'
    name = 'imu_offsets.txt'    
    path += name
    
    talker = imu_talker(path)
    talker.start()    

if __name__ == '__main__':
    main(sys.argv) 


