#! /usr/bin/env python2
import roslib
import rospy
import sys
import cv2
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image, CameraInfo


class usb_cams_talker():
#     RIGHT_CAMERA = 0
#     LEFT_CAMERA = 2
    chanels = {'right':0, 'left':2}
    K = np.array([[7.43335820e+03, 0.00000000e+00, 4.09022733e+02],
                  [0.00000000e+00, 7.92051299e+03, 2.73723391e+02],
                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    t = np.zeros((3,1))
    #9.7 cm
    B = 0.097
#     P = np.ones((3,4), dtype = 'float64').flatten()
    
    def __init__(self):
        self.caps = [cv2.VideoCapture(self.chanels[k]) for k in self.chanels]
        self.topic_names = self.chanels.keys()

        self.seq = 0
        self.frame_id = rospy.get_param('~frame_id', '/camera_link')
        
        #self.cam_image_msg = Image()
        #self.cam_image_msg.is_bigendian = 1
        #self.cam_info_msg = CameraInfo()

        self.pubs_cam_image = []      
        for name in self.topic_names:
            pub = rospy.Publisher("{}/image_raw".format(name), Image, queue_size = 10)
            self.pubs_cam_image.append(pub) 

        self.pubs_cam_info = []
        for name in self.topic_names:
            pub = rospy.Publisher("{}/image_info".format(name), CameraInfo, queue_size = 10)
            self.pubs_cam_info.append(pub)        

        #self.log_info = ""
        self.fps = max([c.get(cv2.CAP_PROP_FPS) for c in self.caps])
        self.loop_rate = rospy.Rate(self.fps)
        self.failure_counter = 0
        #self.loop_rate= rospy.Rate(1)    

    def publisher(self, cap, image_pub, camera_pub, label):
        ret, frame = cap.read()        
        if ret:            
            self.failure_counter = 0

            image_msg = Image()
            image_msg= CvBridge().cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = rospy.Time.now()
            image_msg.header.seq = self.seq
            image_msg.header.frame_id = self.frame_id               
            try:
                image_pub.publish(image_msg)
            except Exception as e:
                rospy.loginfo("An exception of type {} occured. Arguments:\n{}\n{}".format(type(e).__name__, e.args), label)
            
            #publish camera info
            info_msg = CameraInfo()
            info_msg.header.stamp = rospy.Time.now()
            info_msg.header.seq = self.seq
            info_msg.header.frame_id = self.frame_id
            
            width = frame.shape[1]
            height = frame.shape[0]
            info_msg.height = height
            info_msg.width = width
            info_msg.K = self.K.flatten()

            if label == 'right':
                self.t[0,0] = -self.K[0,0] * self.B
            
            info_msg.P = np.hstack((self.K, self.t)).flatten()               
            camera_pub.publish(info_msg)
            return 0

        else:
            self.cap.release()
            return 1
            # cv2.destroyAllWindows()
            # info = "Failed to connect camera, label:{}".format(label)
            # self.cap = cv2.VideoCapture(self.camera)
            # if self.failure_counter > 10: 
            #     rospy.signal_shutdown(info)
            # self.failure_counter += 1
            # rospy.loginfo(info)
            # rospy.Rate(2).sleep()
    
    def talker(self):
        for c, p1, p2, l in zip(self.caps, self.pubs_cam_image, self.pubs_cam_info, self.chanels.keys()):
            err = self.publisher(c, p1, p2, l)
            if err:
               cv2.destroyAllWindows()
               info = "Failed to connect camera, label:{}".format(l)
               if self.failure_counter > 10:
                   rospy.signal_shutdown(info)
               self.failure_counter += 1
               self.caps[c] = cv2.VideoCapture(self.chanels[l])
               rospy.loginfo(info) 

        
    def start(self):
        while not rospy.is_shutdown():
            self.talker()
            self.seq += 1
            self.loop_rate.sleep()
#         try:
#             self.talker()
#             self.seq += 1
#             rospy.spin()
#         except KeyboardInterrupt:
#             print("Shutting down")
#         cv2.destroyAllWindows()
            

def main():
    rospy.init_node('cameras_talker', anonymous = True)
    #default = None
    #camera = int(rospy.get_param('~camera', default))
    #print(camera)
    
 
    talker = usb_cams_talker()
    talker.start()


if __name__ == '__main__':
    main()     
