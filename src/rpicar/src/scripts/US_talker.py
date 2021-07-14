#! /usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Range

from rpicar.msg import telemetry
from library.US_lib import US


class US_talker():
    def __init__(self, US, US_label):
        self.US = US
        self.US.setup_US_ports()
        
        self.US_label = US_label
        self.pub_name = "US{:s}_data".format(self.US_label)
        #print(self.US_label)
#         self.pub = rospy.Publisher(self.pub_name, Float32, queue_size = 10)
#         self.data = 0.0
        #self.sub = rospy.Subscriber("back_range", Range, self.callback, queue_size = 10)
        
        self.range_msg = Range()
        self.init_msg(self.range_msg)
        
        self.telem_msg = telemetry()

        if US_label == "1":
            self.init_msg(self.telem_msg.US1)
        elif US_label == "2":
            self.init_msg(self.telem_msg.US2)

        self.range_msg.header.frame_id = rospy.get_param('~frame_id', 'us_link')
        
        self.range_pub = rospy.Publisher(self.pub_name, Range, queue_size = 10)
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.callback)
        self.telem_pub = rospy.Publisher("telemetry_chatter", telemetry, queue_size = 10)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.05)
        
        
        self.seq = 0

    def callback(self, msg):
        #print("h")
        self.telem_msg = msg

        # if self.US_label == "1":  
        #     self.range_talker(self.telem_msg.US1)
        # elif self.US_label == "2":
        #     self.range_talker(self.telem_msg.US2)                      
        # self.telem_pub.publish(self.telem_msg)

    
    def init_msg(self, msg):
        msg.radiation_type = 0
        # 15 deg = 0.261799 rad
        msg.field_of_view = 0.261799
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = 0.0
    
    def range_talker(self, msg):
        msg.header.stamp = rospy.Time.now()
        try:
            msg.range = self.US.get_distance() * 1e-2
        except Exception as e:
            if self.US.get_distance() == None:
                rospy.loginfo("Error connecting to US{}".format(self.US_label))
            else:
                rospy.loginfo("An exception of type {} occured. Arguments:\n{}".format(type(e).__name__, e.args))

        msg.header.seq = self.seq
        #pub.publish(msg)
        #self.log_info = "Back distance from US#{:s}, {:.2f} [m]".format(self.US_label, self.data)   
        
          
    def start(self):
        self.telem_pub.publish(self.telem_msg)
        while not rospy.is_shutdown():
            self.range_talker(self.range_msg)
            self.range_pub.publish(self.range_msg)
            
            if self.US_label == "1":  
                self.range_talker(self.telem_msg.US1)
            elif self.US_label == "2":
                self.range_talker(self.telem_msg.US2)           
            
            self.telem_pub.publish(self.telem_msg)
            #rospy.loginfo(self.log_info)
            self.seq += 1
            self.loop_rate.sleep()
            
        

def main(args):
#     print(args[1])   
#     trig = 22
#     echo = 17
#     label = '1'    
#    a = args[1]

    rospy.init_node('US', anonymous = True)
    default_label = None

    label = str(rospy.get_param('~US_number', default_label))
    #rosrun rpicar US_talker.py _US_number:=1
    #print(label)
    try:
        if label == '1':
            trig = 22
            echo = 17
        elif label == '2':
            trig = 24
            echo = 23
        else:
            #rospy.signal_shutdown("Invalid US number")
            return
    except Exception as e:
            template = "An exception of type {0} occured. Arguments:\n{1!r}"
            message = template.format(type(e).__name__, e.args)
            print(message)
            return
            
    US_ = US(trig = trig, echo = echo)
    talker = US_talker(US = US_, US_label = label)
    talker.start()    


if __name__ == '__main__':
    main(sys.argv)   
#     rospy.init_node('US_2', anonymous = True)   
#     US2 = US(trig = 24, echo = 23)
#     rg2 = range_proc(US = US2, US_label = '2')
#     rg2.start()
#     
