#! /usr/bin/env python3

import roslib
import rospy
import sys
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import numpy as np

class bridge():
    def __init__(self):
        self.pub = rospy.Publisher("vo", Odometry, queue_size = 10)
        self.sub = rospy.Subscriber("mono_odometer/odometry", Odometry, self.callback, queue_size = 10) 
        self.cov = list(np.eye(6).flatten())
    def callback(self, msg):
        _msg = msg
         
        #_msg.pose.pose = msg.pose
        _msg.pose.covariance = self.cov

        #_msg.twist.pose = msg.pose
        _msg.twist.covariance = self.cov
        
        self.loop_rate= rospy.Rate(1)
                
        self.pub.publish(_msg)
                 

def main():
    rospy.init_node('vo_cov_bridge', anonymous = True)
    bridge()
    rospy.spin()


if __name__ == '__main__':
    main()   