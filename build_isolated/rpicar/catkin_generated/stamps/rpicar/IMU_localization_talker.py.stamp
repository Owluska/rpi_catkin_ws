#! /usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Imu, MagneticField, Temperature


from library.mpu9250_lib import mpu9250

import numpy as np
import math
import ahrs
from ahrs import Quaternion

from time import sleep, time

class imu_talker():
    def __init__(self):
        self.offsets = np.array([[0.9964611434970413, -0.1358959565043638],
                                     [0.998331379179552,0.06877502271143854],
                                     [-0.9860600182267061,-0.3923817524528095],
                                      1.7522125244140625,-2.4026947021484375,-0.3843841552734375,
                                     [145.3125],[35.3759765625],[8.349609375]], dtype = 'object')
        self.imu = mpu9250()
        
        self.ax = 0.00
        self.ay = 0.00
        self.az = 10.00
        
        self.gx = 0.00
        self.gy = 0.00
        self.gz = 0.00
        
        self.mx = 0.00
        self.my = 0.00
        self.mz = 0.00

        self.temp = 0.0

        self.dt = 0.00
        self.t = 0.00

        self.acc_data = np.array([self.ax, self.ay, self.az])
        self.gyro_data = np.array([self.gx, self.gy, self.gz]) 
        self.t_step = 0.03  

        self.g = 9.84
        self.D2R = math.pi/180
        self.R2D = 180 /math.pi

        self.pose = np.array([.0, .0, .0])
        self.orientation = {'roll':0.0, 'pitch':.0, 'yaw':.0}
        
        self.acc_data = np.array([[.0, .0, self.g]])
        self.gyro_data = np.array([[.0, .0, .0]])
        self.mag_data = np.array([[.0,.0,.0]])
        self.quat_data = np.array([[1.0, .0, .0, 0.0]])
        self.use_magnetometer = False
        if self.use_magnetometer:  
            #self.filter = ahrs.filters.Madgwick(acc = self.acc_data, gyr = self.gyro_data, mag = self.mag_data)
            self.filter = ahrs.filters.Complementary(acc = self.acc_data, gyr = self.gyro_data, mag = self.mag_data, frequency = 1/self.t_step)
        else:
            #self.filter = ahrs.filters.Madgwick(acc = self.acc_data, gyr = self.gyro_data)
            self.filter = ahrs.filters.Complementary(acc = self.acc_data, gyr = self.gyro_data, frequency = 1/self.t_step)
        
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

        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.05)

    def get_mpu9250_data(self):
            self.ax,self.ay,self.az,self.gx,self.gy,self.gz = self.imu.mpu6050_conv() # read and convert mpu6050 data
            self.mx,self.my,self.mz = self.imu.AK8963_conv()
            return np.array([self.ax,self.ay,self.az,self.gx,self.gy,self.gz,self.mx,self.my,self.mz])
        
    def calibrated_mpu9250(self):
        raw  = self.get_mpu9250_data()
         
        mpu_cal = np.zeros_like(raw)
        cal_rot_indicies = [[6,7],[7,8],[6,8]] # heading indices
        for i in range(3):   
            mpu_cal[i] = self.offsets[i][0]*raw[i]+ self.offsets[i][1]
        for i in range(3,6):
            mpu_cal[i] = raw[i] - self.offsets[i]
        for i in range(6,9):
            j = i-6
            mpu_cal[i] = raw[i] - self.offsets[cal_rot_indicies[j][0]]
        self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.mx, self.my, self.mz = mpu_cal
        return mpu_cal
    
    def sensor_data_to_AHRS(self):
        self.ax *= self.g
        self.ay *= self.g
        self.az *= self.g

        self.gx *= self.D2R
        self.gy *= self.D2R
        self.gz *= self.D2R

        acc_data = np.array([[self.ax, self.ay, self.az]])
        self.acc_data = np.append(self.acc_data, acc_data, axis = 0)

        gyro_data = np.array([[self.gx, self.gy, self.gz]])
        self.gyro_data = np.append(self.gyro_data, gyro_data, axis = 0)

        mag_data = np.array([[self.mx, self.my, self.mz]])
        self.mag_data = np.append(self.mag_data, mag_data, axis = 0)
    
    def read_sensor_data(self):
        try:
            _ = self.calibrated_mpu9250()
            self.sensor_data_to_AHRS()
            self.temp = self.imu.read_temp()
        except Exception as e:
            rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
        

    
    def talker(self, msg):   
        self.read_sensor_data()
        
        #publish imu acceleration and gyro
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.seq = self.seq
        self.imu_msg.angular_velocity.x  = self.gx
        self.imu_msg.angular_velocity.y = self.gy
        self.imu_msg.angular_velocity.z = self.gz
        
        self.imu_msg.linear_acceleration.x = self.ax
        self.imu_msg.linear_acceleration.y = self.ay
        self.imu_msg.linear_acceleration.z = self.az
        self.pub_raw.publish(self.imu_msg)
        
        #publish magnetometer
        self.mag_msg.header.stamp = rospy.Time.now()
        self.mag_msg.header.seq = self.seq
        self.mag_msg.magnetic_field.x  = self.mx
        self.mag_msg.magnetic_field.y = self.my
        self.mag_msg.magnetic_field.z = self.mz
        
        self.pub_mag.publish(self.mag_msg)
        
        #publish temperature
        self.temp_msg.header.stamp = rospy.Time.now()
        self.temp_msg.header.seq = self.seq
        self.temp_msg.temperature = self.temp
        
        self.pub_temp.publish(self.temp_msg)

    def get_attitude(self):
        if self.use_magnetometer:
            #self.filter.updateMARG(self.quat_data[-1], gyr = self.gyro_data[-1], acc = self.acc_data[-1], mag = self.mag_data[-1])
            pass      
        else:
            #self.filter.updateIMU(self.quat_data[-1], gyr = self.gyro_data[-1], acc = self.acc_data[-1])
            self.filter.attitude_propagation(self.quat_data[-1], omega = self.gyro_data[-1])
            quat_data = self.filter.update(self.quat_data[-1], gyr = self.gyro_data[-1], acc = self.acc_data[-1])
            #print(self.quat_data)
        
        quat_data = quat_data.reshape((1, 4))
        self.quat_data = np.append(self.quat_data, quat_data, axis = 0)
        #self.quat_data = self.filter.Q
        angles = Quaternion(self.quat_data[-1]).to_angles()
        angles *= self.R2D
        for k, a in zip(self.orientation, angles): 
            self.orientation[k] = float(a)

    def start(self):
        tt = time()
        while not rospy.is_shutdown():
            self.dt = time() - tt
            tt = time()
            self.t += self.dt

            self.read_sensor_data()
            
            if self.publish:
                self.talker()

            self.get_attitude()

             
            rospy.set_param("orientation", self.orientation)
            
            if self.t % 1 <= 0.15:
                data = self.orientation
                template = "{:.2f}s {:.2f} {:.2f} {:.2f}".format(self.t, *data.values())   
                print(template)

            sleep(self.t_step)
            self.seq += 1
            #self.loop_rate.sleep()

def main(args):
    rospy.init_node('imu_talker', anonymous = True)   
    talker = imu_talker()
    talker.start()    


if __name__ == '__main__':
    main(sys.argv) 


