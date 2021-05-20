#! /usr/bin/env python3
import roslib
import rospy
import sys
import cv2
from 
import numpy as np

from sensor_msgs.msg import Image, CameraInfo


class usb_cam_talker():
#     RIGHT_CAMERA = 0
#     LEFT_CAMERA = 2
    camera_labels = {'right':0, 'left':2}
    K = np.array([[7.43335820e+03, 0.00000000e+00, 4.09022733e+02],
                  [0.00000000e+00, 7.92051299e+03, 2.73723391e+02],
                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    t = np.zeros((3,1))
    #9.7 cm
    B = 0.097
#     P = np.ones((3,4), dtype = 'float64').flatten()
    
    def __init__(self, camera):
        self.cap = cv2.VideoCapture(camera)              
        self.topic_name = [k for k, v in self.camera_labels.items() if v == camera][0]
        self.seq = 0
        self.frame_id = rospy.get_param('~frame_id', '/camera_link')
        
        self.cam_raw_msg = Image()
        self.cam_info_msg = CameraInfo()
        
        self.pub_cam_raw = rospy.Publisher("{}/image_raw".format(self.topic_name), Image, queue_size = 10)
        self.pub_cam_info = rospy.Publisher("{}/image_info".format(self.topic_name), CameraInfo, queue_size = 10)
        self.log_info = ""
        self.loop_rate= rospy.Rate(1)    

    def talker(self):
        ret, frame = self.cap.read()
        
        if ret:
            try:
                #publish camera image
                self.cam_raw_msg.header.stamp = rospy.Time.now()
                self.cam_raw_msg.header.seq = self.seq
                self.cam_raw_msg.header.frame_id = self.frame_id
                
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                width = frame.shape[1]
                height = frame.shape[0]
                
                self.cam_raw_msg.data = np.asarray(image, np.uint8)
                #print(self.cam_raw_msg.data)
                self.cam_raw_msg.height = height
                self.cam_raw_msg.width = width
                self.pub_cam_raw.publish(self.cam_raw_msg)
                
                #publish camera info
                self.cam_info_msg.header.stamp = rospy.Time.now()
                self.cam_info_msg.header.seq = self.seq
                self.cam_info_msg.header.frame_id = self.frame_id
                
                self.cam_info_msg.height = height
                self.cam_info_msg.width = width
                self.cam_info_msg.K = self.K.flatten()
                #print(self.cam_info_msg.K)
                if self.topic_name == 'right':
                    self.t[0,0] = -self.K[0,0] * self.B
                
                self.cam_info_msg.P = np.hstack((self.K, self.t)).flatten()
                #print(self.cam_info_msg.P)
                
                self.pub_cam_info.publish(self.cam_info_msg)
            except:
                pass
        else:
            self.cap.release()
            cv2.destroyAllWindows()
            rospy.loginfo("Failed to connect camera, label:{}".format(self.topic_name))
            
        
        
    def start(self):
        while not rospy.is_shutdown():
            self.talker()
            self.seq += 1
            self.loop_rate.sleep()

def main(args):
    rospy.init_node('camera_talker', anonymous = True)
    default = None
    camera = rospy.get_param('~camera', default)
    print(camera)
    
    if camera != None:
        talker = usb_cam_talker(camera)
        talker.start()
    else:
        rospy.loginfo("Invalid camera channel")
        rospy.signal_shutdown("Invalid camera channel")
        return 

if __name__ == '__main__':
    main(sys.argv)     
