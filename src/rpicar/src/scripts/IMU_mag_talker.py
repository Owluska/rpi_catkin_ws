#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import MagneticField

from library.mpu9250_lib import mpu9250
from rpicar.msg import telemetry
import numpy as np

class imu_mag_talker():
    def __init__(self):
        self.imu = mpu9250()

        self.offsets = np.array([[0.9964611434970413, -0.1358959565043638],
                                [0.998331379179552,0.06877502271143854],
                                [-0.9860600182267061,-0.3923817524528095],
                                1.7522125244140625,-2.4026947021484375,-0.3843841552734375,
                                [145.3125],[35.3759765625],[8.349609375]], dtype = 'object')

        self.ax = None
        self.ay = None
        self.az = None
        
        self.gx = None
        self.gy = None
        self.gz = None
        
        self.mx = None
        self.my = None
        self.mz = None  
        self.mx = None
        self.my = None
        self.mz = None

        self.var_m = np.array([5.774, 1.119, 1.466])
        self.I = np.eye(3)

        self.mag_msg = MagneticField()
        self.mag_msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link')
        
        self.telem_msg = telemetry()
        
        self.seq = 0
        self.temp_pub = rospy.Publisher("imu/mag", MagneticField, queue_size = 10)
        self.telem_pub = rospy.Publisher("telemetry_chatter", telemetry, queue_size = 10)
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.callback)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.1)
    
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
        self.ax,self.ay,self.az,self.gx,self.gy,self.gz,self.mx,self.my,self.mz = mpu_cal
        return mpu_cal
    
    def read_sensor_data(self):
        try:
            _ = self.calibrated_mpu9250()
        except Exception as e:
            rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
    
    
    def talker(self, msg):     
        #publish temperature
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.seq
        msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link') 
        
        msg.magnetic_field.x = self.mx
        msg.magnetic_field.y = self.my
        msg.magnetic_field.z = self.mz
        msg.magnetic_field_covariance = (self.I * self.var_m).flatten()
        
        
    def callback(self, msg):
        self.telem_msg = msg

    def start(self):
        while not rospy.is_shutdown():
            self.read_sensor_data()
            
            self.talker(self.mag_msg)
            self.temp_pub.publish(self.mag_msg)
            
            self.talker(self.telem_msg.IMU_mag)
            self.telem_pub.publish(self.telem_msg)
            
            self.seq += 1
            self.loop_rate.sleep()

def main():
    rospy.init_node('imu_mag_talker', anonymous = True)   
    talker = imu_mag_talker()
    talker.start()    


if __name__ == '__main__':
    main() 