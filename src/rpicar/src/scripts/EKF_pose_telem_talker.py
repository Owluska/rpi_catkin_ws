#! /usr/bin/env python3
import rospy

from rpicar.msg import telemetry
from geometry_msgs.msg import PoseWithCovarianceStamped

class EKF_telem_bridge():
    def __init__(self):     
        self.pose_msg = PoseWithCovarianceStamped()       
        self.telem_msg = telemetry()
        
        self.seq = 0
        self.pose_sub = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.pose_callback)
        
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.telem_callback)
        self.telem_pub = rospy.Publisher("telemetry_chatter", telemetry, queue_size=10)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.1)
    
    
    def telem_callback(self, msg):
        if rospy.is_shutdown():
            return     
        self.telem_msg = msg

    
    def pose_callback(self, msg):
        if rospy.is_shutdown():
            return      
        self.pose_msg = msg
  

    def start(self):
        while not rospy.is_shutdown():
            self.telem_msg.EKF_pose = self.pose_msg
            self.telem_pub.publish(self.telem_msg)
            
            self.seq += 1
            self.loop_rate.sleep()

def main():
    rospy.init_node('ekf_bridge', anonymous = True)   
    talker = EKF_telem_bridge()
    talker.start()    


if __name__ == '__main__':
    main() 