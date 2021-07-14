#! /usr/bin/env python3
import roslib
import rospy
import sys
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

from library.mpu9250_lib import mpu9250
from rpicar.msg import telemetry
import numpy as np

class imu_temp_talker():
    def __init__(self):
        self.imu = mpu9250()  
        self.temp = None
          
        
        self.temp_msg = Temperature()
        self.temp_msg.header.frame_id = rospy.get_param('~frame_id', 'imu_link')
        
        self.telem_temp_msg = telemetry().IMU_temp
        self.temp_msg.header.frame_id = 'imu_temp_link'
        
        self.seq = 0
        self.pub_temp = rospy.Publisher("imu/temp", Temperature, queue_size = 10)
        self.telem_pub_temp = rospy.Publisher("telemetry_chatter", Temperature, queue_size = 10)

        self.log_info = ""
        self.loop_rate= rospy.Rate(1)
    
    def read_sensor_data(self):
        try:
            self.temp = self.imu.read_temp()
        except Exception as e:
            rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
    
    
    def talker_temp(self, msg, pub):     
        #publish temperature
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.seq
        msg.temperature = self.temp
        
        pub.publish(msg)
        
    def start(self):
        while not rospy.is_shutdown():
            self.read_sensor_data()
            
            self.talker_temp(self.temp_msg, self.pub_temp)
            self.talker_temp(self.telem_temp_msg, self.telem_pub_temp)
            
            self.seq += 1
            self.loop_rate.sleep()

def main(args):
    rospy.init_node('imu_talker', anonymous = True)   
    talker = imu_temp_talker()
    talker.start()    


if __name__ == '__main__':
    main(sys.argv) 