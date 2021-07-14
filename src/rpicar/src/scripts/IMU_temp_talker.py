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
        
        self.telem_msg = telemetry()
        
        self.seq = 0
        self.temp_pub = rospy.Publisher("imu/temp", Temperature, queue_size = 10)
        self.telem_pub = rospy.Publisher("telemetry_chatter", telemetry, queue_size = 10)
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.callback)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.1)
    
    def read_sensor_data(self):
        try:
            self.temp = self.imu.read_temp()
        except Exception as e:
            rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
    
    
    def talker(self, msg):     
        #publish temperature
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.seq
        msg.temperature = self.temp
        
        
    def callback(self, msg):
        self.telem_msg = msg

    def start(self):
        while not rospy.is_shutdown():
            self.read_sensor_data()
            
            self.talker(self.temp_msg)
            self.temp_pub.publish(self.temp_msg)
            
            #print(self.temp_msg)
            self.talker(self.telem_msg.IMU_temp)
            self.telem_pub.publish(self.telem_msg)
            
            self.seq += 1
            self.loop_rate.sleep()

def main():
    rospy.init_node('imu_talker', anonymous = True)   
    talker = imu_temp_talker()
    talker.start()    


if __name__ == '__main__':
    main() 