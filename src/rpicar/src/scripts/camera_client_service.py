#! /usr/bin/env python3
#import sys
import rospy
#from __future__ import print_function
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from rpicar.srv import camera
import rospy

class camera_service():
    def __init__(self, label):
        self.label = label
        self.sub_name = "/{}/image_info".format(self.label)
        self.serv_name = "/{}".format(self.label)
        #print(self.sub_name, self.serv_name)
        self.sub = rospy.Subscriber(self.sub_name, CameraInfo, self.callback, queue_size = 1)
        self.msg = CameraInfo()
        self.loop_rate = rospy.Rate(1)  

    def callback(self, msg):
        print("get_data")
        self.msg = msg

    def camera_client(self):
        rospy.wait_for_service(self.serv_name)
        try:
            camera_handle = rospy.ServiceProxy(self.serv_name, SetCameraInfo)
            camera_handle(self.msg)
        except Exception as e:
            #template = "An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args)
            template = "Service call failed, {}".format(e)
            rospy.loginfo(template)

    def handle_camera(self, req):
        # req.success = True
        # req.status = self.label   
        return [True, '']

    def start_camera_server(self):
        
        #while not rospy.is_shutdown():
        rospy.Service(self.serv_name, camera, self.handle_camera)
        rospy.spin()
        #self.camera_client()
        #self.loop_rate.sleep()

def main():
    rospy.init_node('camera_server')
    default = None    
    label = str(rospy.get_param('~label', default))
    #print(label)
    
    if camera != None:
        cam = camera_service(label)
        cam.start_camera_server()
    else:
        rospy.loginfo("Invalid camera channel")
        rospy.signal_shutdown("Invalid camera channel")
        return

if __name__ == '__main__':
    main()
    