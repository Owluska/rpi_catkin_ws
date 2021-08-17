#! /usr/bin/env python3
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Image, CameraInfo



def handle_camera_pose(msg):
    # (x, y, z)
    translation = (0.18, 0.05, 0.05)
    #(x, y, z, w)
    rotation = (0, 0, 0, 1)
    parent = "base_link"
    child = "/usb_camera/"
    camera_name = "/usb_camera/"
    br = tf.TransformBroadcaster()
    
    br.sendTransform(translation, rotation, rospy.Time(), parent, child)
    

def main():
    rospy.init_node('usb_camera_tf_publisher')
    rospy.Subscriber("/odom", Odometry, handle_camera_pose, queue_size = 10)
    
if __name__ == "__main__":
    main()