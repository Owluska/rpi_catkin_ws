#! /usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Imu


from library.mpu9250_lib import mpu9250
from rpicar.msg import telemetry
import numpy as np

class inertia_talker():
    def __init__(self):
        self.offsets = np.array([[0.9964611434970413, -0.1358959565043638],
                                     [0.998331379179552,0.06877502271143854],
                                     [-0.9860600182267061,-0.3923817524528095],
                                      1.7522125244140625,-2.4026947021484375,-0.3843841552734375,
                                     [145.3125],[35.3759765625],[8.349609375]], dtype = 'object')
        self.imu = mpu9250()
        
        self.ax = None
        self.ay = None
        self.az = None
        
        self.gx = None
        self.gy = None
        self.gz = None
        
        self.mx = None
        self.my = None
        self.mz = None
        
        self.temp = None
        
        self.g = 9.84
        self.D2R = 3.14/180
        
        
        self.I = np.eye(3)

        self.var_w = np.array([0.067, 0.107, 0.029])
        self.var_f = np.array([1.962, 3.31 , 1.603])
        
        self.imu_msg = Imu()
        self.telem_msg = telemetry()

        self.inertia_msg_init(self.imu_msg)
        self.inertia_msg_init(self.telem_msg.IMU)
       
        
        self.seq = 0
        
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.callback)
        self.pub_inertia = rospy.Publisher("imu/raw", Imu, queue_size = 10)
        self.telem_pub = rospy.Publisher("telemetry_chatter", telemetry, queue_size = 10)
        
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
        self.ax,self.ay,self.az,self.gx,self.gy,self.gz,self.mx,self.my,self.mz = mpu_cal
        return mpu_cal
    
    def read_sensor_data(self):
        try:
            _ = self.calibrated_mpu9250()
        except Exception as e:
            rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
        
    def inertia_msg_init(self, msg):
        msg.angular_velocity_covariance = (self.I * self.var_w).flatten()
        msg.linear_acceleration_covariance = (self.I * self.var_f).flatten()
        msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link')
    
    def callback(self, msg):
        self.telem_msg = msg
        self.talker(self.telem_msg.IMU)
        self.telem_pub.publish(self.telem_msg)
    
    def talker(self, msg):   
        #publish imu acceleration and gyro
        msg.header.stamp = rospy.Time.now()
        
        msg.header.seq = self.seq
        msg.angular_velocity.x  = self.gx
        msg.angular_velocity.y = self.gy
        msg.angular_velocity.z = self.gz
        
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az
        #pub.publish(msg)

        
    def start(self):
        self.telem_pub.publish(self.telem_msg)
        while not rospy.is_shutdown():
            self.read_sensor_data()
            
            self.talker(self.imu_msg)
            self.pub_inertia.publish(self.imu_msg)

            # self.talker(self.telem_msg.IMU)
            # #print(self.telem_msg)
            # self.telem_pub.publish(self.telem_msg)
            
            self.seq += 1
            self.loop_rate.sleep()

def main(args):
    rospy.init_node('imu_talker', anonymous = True)   
    talker = inertia_talker()
    talker.start()    


if __name__ == '__main__':
    main(sys.argv) 


