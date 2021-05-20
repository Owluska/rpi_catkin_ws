#! /usr/bin/env python3

import roslib
import rospy
import sys
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from datetime import datetime

import wiringpi
from time import time, sleep

class US:
    def __init__(self, trig = 24, echo = 23):
        self.US = None
        self.US_TRIG = trig
        self.US_ECHO = echo
        self.T = 25
        self.sound_vel=(331.5+0.6*self.T)
        self.us = 1e-6
        self.ms = 1e-3
        self.timeout = 10 * self.ms
        self.dist = None        

    def setup_US_ports(self):
        wiringpi.wiringPiSetupGpio()
        wiringpi.pinMode(self.US_ECHO, wiringpi.GPIO.INPUT)
    
        wiringpi.pinMode(self.US_TRIG, wiringpi.GPIO.OUTPUT)
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.LOW)

    def get_distance(self):
        sleep(0.38)
        pulse_start = 0
        pulse_end = 0
        
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.LOW)
        sleep(0.05)
        
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.HIGH)
        sleep(10*self.us)
        wiringpi.digitalWrite(self.US_TRIG, wiringpi.GPIO.LOW)
        
        start_time = time()
        while(wiringpi.digitalRead(self.US_ECHO) == wiringpi.GPIO.LOW):
            pulse_start = time()
            if(pulse_start - start_time > self.timeout):
                break
            
        start_time = time()
        while(wiringpi.digitalRead(self.US_ECHO) == wiringpi.GPIO.HIGH):
            pulse_end = time()
            if(pulse_end - start_time > self.timeout):
                break
        
     
        distance = self.sound_vel * (pulse_end - pulse_start) *0.5*100
        distance = abs(round(distance, 2))
        
        if distance < 0 or distance > self.sound_vel * (self.timeout) * 100:
                distance = None            
        self.dist = distance
        #in cm
        return distance 

class US_talker():
    def __init__(self, US, US_label):
        self.US = US
        self.US.setup_US_ports()
        
        self.US_label = US_label
        self.pub_name = "US{:s}_data".format(self.US_label)
        
#         self.pub = rospy.Publisher(self.pub_name, Float32, queue_size = 10)
#         self.data = 0.0
        #self.sub = rospy.Subscriber("back_range", Range, self.callback, queue_size = 10)
        
        self.range_msg = Range()
        self.range_msg.radiation_type = 0
        # 15 deg = 0.261799 rad
        self.range_msg.field_of_view = 0.261799
        self.range_msg.min_range = 0.02
        self.range_msg.max_range = 4.0
        self.range_msg.range = 0.0
        self.range_msg.header.frame_id = rospy.get_param('~frame_id', 'us_link')
        
        self.range_pub = rospy.Publisher(self.pub_name, Range, queue_size = 10)
        self.log_info = ""
        self.loop_rate= rospy.Rate(1)
        
        
        self.seq = 0

            
    def range_talker(self):
        self.range_msg.header.stamp = rospy.Time.now()
        self.range_msg.range = self.US.get_distance() * 1e-2
        self.range_msg.header.seq = self.seq
        self.range_pub.publish(self.range_msg)
        #self.log_info = "Back distance from US#{:s}, {:.2f} [m]".format(self.US_label, self.data)   
        rospy.sleep(1)
          
    def start(self):
        while not rospy.is_shutdown():
            self.range_talker() 
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
            rospy.loginfo(message)
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
