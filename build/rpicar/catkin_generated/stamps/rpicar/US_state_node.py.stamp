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
        
        self.pub = rospy.Publisher(self.pub_name, Float32, queue_size = 10)
        self.data = 0.0
#         self.sub = rospy.Subscriber("back_range", Range, self.callback, queue_size = 10)
        
#         self.range = Range()
#         self.range.radiation_type = 0
#         # 15 deg = 0.261799 rad
#         self.range.field_of_view = 0.261799
#         self.range.min_range = 0.1
#         self.range.max_range = 4.0
#         self.range.range = 0.0
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(1)
        

            
    def range_talker(self):
        self.data = self.US.get_distance() * 1e-2
        #self.log_info = "Back distance from US#{:s}, {:.2f} [m]".format(self.US_label, self.data)   
        rospy.sleep(1)
          
    def start(self):
        while not rospy.is_shutdown():
            self.range_talker() 
            #rospy.loginfo(self.log_info)
            self.pub.publish(self.data)
            self.loop_rate.sleep()
            
        

def main(args):
#     print(args[1])   
#     trig = 22
#     echo = 17
#     label = '1'    
    a = args[1]
    if a == '1':
        trig = 22
        echo = 17
        label = '1'
    elif a == '2':
        trig = 24
        echo = 23
        label = '2'
    else:
        #rospy.signal_shutdown("Invalid US number")
        return

    rospy.init_node('US', anonymous = True)   
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
