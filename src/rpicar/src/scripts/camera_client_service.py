#! /usr/bin/env python3
#import sys
import rospy
from __future__ import print_function
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from rpicar.srv import camera
import rospy

def camera_client(service_name):
    rospy.wait_for_service('camera')
    try:
        camera_handle = rospy.ServiceProxy('camera', SetCameraInfo)
        camera_handle()
    except Exception as e:
        #template = "An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args)
        template = "Service call failed, {:s}".format(e)
        rospy.loginfo(template)

def handle_camera(req):
    return req

def camera_server():
    rospy.init_node('camera_server')
    default = None    
    camera = rospy.get_param('~camera', default)
    print(camera)
    
    if camera != None:
        if camera == 0:
            service_name = "left_camera"
        elif camera == 2:
            service_name = "right_camera"
        server = rospy.Service(service_name, camera, handle_camera)

    else:
        rospy.loginfo("Invalid camera channel")
        rospy.signal_shutdown("Invalid camera channel")
        return
    rospy.spin() 
    
if __name__ == '__main__':
    camera_server()
    camera_client()
    