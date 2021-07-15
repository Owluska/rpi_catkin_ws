#! /usr/bin/env python3
import rospy

from rpicar.msg import telemetry
from nav_msgs.msg import Odometry

class EKF_telem_bridge():
    def __init__(self):     
        self.vo_msg = Odometry()       
        self.telem_msg = telemetry()
        
        self.seq = 0
        self.pose_sub = rospy.Subscriber("vo", Odometry, self.vo_callback)
        
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.telem_callback)
        self.telem_pub = rospy.Publisher("telemetry_chatter", telemetry, queue_size=10)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.1)
    
    
    def telem_callback(self, msg):
        if rospy.is_shutdown():
            return     
        self.telem_msg = msg

    
    def vo_callback(self, msg):
        if rospy.is_shutdown():
            return      
        self.vo_msg = msg
  

    def start(self):
        while not rospy.is_shutdown():
            self.telem_msg.VO = self.vo_msg
            self.telem_pub.publish(self.telem_msg)
            
            self.seq += 1
            self.loop_rate.sleep()

def main():
    rospy.init_node('vo_bridge', anonymous = True)   
    talker = EKF_telem_bridge()
    talker.start()    


if __name__ == '__main__':
    main() 