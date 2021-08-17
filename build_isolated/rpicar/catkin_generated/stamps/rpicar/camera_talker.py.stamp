#! /usr/bin/env python
import roslib
import rospy
import sys
import cv2
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image, CameraInfo


class usb_cam_talker():
    K = np.array([[4486.418534, 0.000000, -144.909998],
                  [0.000000, 4649.541611, 34.559013],
                  [0.000000, 0.000000, 1.000000]])
    t = np.zeros((3,1))
    width = 640
    height = 480
    distortion = [-3.666348, 48.962889, 0.016858, 0.148182, 0.000000]
    rectification = np.eye(3)
    projection = np.array([[4580.996094, 0.000000, -146.452712, 0.000000],
                          [0.000000, 4617.534668, 34.222443, 0.000000],
                          [0.000000, 0.000000, 1.000000, 0.000000]])
    
#     P = np.ones((3,4), dtype = 'float64').flatten()
    
    def __init__(self, camera):
        self.camera = camera
        self.camera_name = "/usb_camera"
        #print(self.camera)
        self.cap = cv2.VideoCapture(self.camera) 
        #self.bridge = CvBridge()
        self.seq = 0
        self.frame_id = rospy.get_param('~frame_id', self.camera_name)
        
        self.cam_raw_msg = Image()
        self.cam_raw_msg.is_bigendian = 1
        self.cam_info_msg = CameraInfo()
        
        self.pub_cam_raw = rospy.Publisher(self.camera_name+"/image_raw", Image, queue_size = 10)
        self.pub_cam_info = rospy.Publisher(self.camera_name+"/camera_info", CameraInfo, queue_size = 10)
        self.log_info = ""
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.loop_rate= rospy.Rate(self.fps)
        self.failure_counter = 0
        #self.loop_rate= rospy.Rate(1)    

    def talker(self):
        ret, frame = self.cap.read()        
        if ret:            
                self.failure_counter = 0

                self.cam_raw_msg = CvBridge().cv2_to_imgmsg(frame, "bgr8")
                time = rospy.Time.now()
                self.cam_raw_msg.header.stamp = time
                self.cam_raw_msg.header.seq = self.seq
                self.cam_raw_msg.header.frame_id = self.frame_id
                self.cam_raw_msg.height = self.height
                self.cam_raw_msg.width = self.width

                #publish camera info
                self.cam_info_msg.header.stamp = time
                self.cam_info_msg.header.seq = self.seq
                self.cam_info_msg.header.frame_id = self.frame_id

                self.cam_info_msg.height = self.height
                self.cam_info_msg.width = self.width
                self.cam_info_msg.K = list(self.K.flatten())
                
                self.cam_info_msg.distortion_model = "plumb_bob"
                self.cam_info_msg.P = self.projection.flatten()
                self.cam_info_msg.D = self.distortion
                self.cam_info_msg.R = self.rectification.flatten()
    

                try:
                    self.pub_cam_raw.publish(self.cam_raw_msg)
                    self.pub_cam_info.publish(self.cam_info_msg)
                except Exception as e:
                    rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))
                
            
            # except Exception as e:
            #     self.cap.release()
            #     cv2.destroyAllWindows()
            #     info = "An exception of type {} occured. Arguments:\n{}, camera label:{}".format(type(e).__name__, e.args, self.topic_name)
            #     self.cap = cv2.VideoCapture(self.camera) 
            #     if self.failure_counter > 10:
            #         rospy.signal_shutdown(info)
            #     self.failure_counter += 1
            #     rospy.loginfo(info)
            #     rospy.Rate(2).sleep()

        else:
            self.cap.release()
            cv2.destroyAllWindows()
            info = "Failed to connect camera, label:{}".format(self.topic_name)
            self.cap = cv2.VideoCapture(self.camera)
            if self.failure_counter > 10: 
                rospy.signal_shutdown(info)
            self.failure_counter += 1
            rospy.loginfo(info)
            rospy.Rate(2).sleep()
        
    def start(self):
        while not rospy.is_shutdown():
            self.talker()
            self.seq += 1
            self.loop_rate.sleep()
            

def main(args):
    rospy.init_node('camera_talker', anonymous = True)
    default = 0
    camera = int(rospy.get_param('~camera', default))
    #print(camera)
    
    if camera != None:
        talker = usb_cam_talker(camera)
        talker.start()
    else:
        rospy.loginfo("Invalid camera channel")
        rospy.signal_shutdown("Invalid camera channel")
        return 

if __name__ == '__main__':
    main(sys.argv)     