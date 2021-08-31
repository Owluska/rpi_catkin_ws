#! /usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Range

# from rpicar.msg import telemetry
from drivers.US_lib import US


class US_talker():
    def __init__(self, US, US_label):
        self.US = US
        self.US.setup_US_ports()
        
        self.US_label = US_label
        self.data = 0.0
        self.pub_name = "US{:s}_data".format(self.US_label)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(0.05)
                
        self.seq = 0
    
        
          
    def start(self):
        while not rospy.is_shutdown():
            try:
                self.data =  self.US.get_distance() * 1e-2
                rospy.set_param("US{:s}_data".format(self.US_label), self.data)
            except Exception as e:
                template = "An exception of type {0} occured. Arguments:\n{1!r}"
                message = template.format(type(e).__name__, e.args)
                rospy.loginfo(message)
        
            
            self.seq += 1
            #self.loop_rate.sleep()
            
        

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
