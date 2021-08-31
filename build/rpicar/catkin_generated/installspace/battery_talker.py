#! /usr/bin/env python3

import roslib
import rospy
import sys
from sensor_msgs.msg import BatteryState
from ina219 import INA219
from time import time, sleep
from datetime import datetime
from drivers.multiplexer import PCA9547
from rpicar.msg import telemetry

class battery_state_proc():
    def __init__(self, adress = 0x41):
        self.ch = 0
        #self.pca = PCA9547()
        #self.pca.set_channel(self.ch)
        
        self.ina = None
        self.adress = adress
        
        self.shunt_res = 0.01
        
        self.voltage = None
        self.current = None
        self.ina = self.setup_ina219(self.adress)
        
        self.pub = rospy.Publisher("battery_state", BatteryState, queue_size = 10)
        self.msg_pub = rospy.Publisher("telemetry_chatter", BatteryState, queue_size = 10)
        self.msg = telemetry()
        #self.sub = rospy.Subscriber("battery_state", BatteryState, self.callback, queue_size = 10)
        
        self.bs = self.battery_init()
        self.msg.Battery = self.battery_init()
        #self.pub.publish(self.bs)
        
        self.log_info = ""
        self.loop_rate= rospy.Rate(1)
        self.seq  = 0

    
    def setup_ina219(self, adress):
        try:
            ina = INA219(self.shunt_res, address = adress)
            ina.configure()
        except Exception as e:
            template = "An exception of type {0} occured. Arguments:\n{1!r}"
            message = template.format(type(e).__name__, e.args)
            self.log_info = message
            ina = None
        return ina
    
    def measure(self):
        if self.ina != None:
            self.voltage = self.ina.voltage()
            self.current = self.ina.current() * 1e-3
            return self.voltage, self.current
        else:
            return None, None
    
    def battery_init(self):
        bs = BatteryState()
        #Capacity in mAh
        bs.design_capacity = 2200
        #battery discharging
        bs.power_supply_status = 2
        #tech. - LiION
        bs.power_supply_technology = 2
        #health unknown
        bs.power_supply_health = 0
        #present
        bs.present = False
        
        bs.voltage = 0.0
        bs.current = 0.0
        
        cell = bs.voltage/2
        bs.cell_voltage = [cell, cell]
        bs.header.frame_id = rospy.get_param('~frame_id', 'bs_link') 
        return bs
        

            
    def battery_talker(self, pub, bs):
        self.voltage, self.current = self.measure()

        if self.voltage != None and self.current != None:
            bs.header.stamp = rospy.Time.now()
            bs.voltage = self.voltage
            bs.current = self.current
            bs.cell_voltage = [self.voltage/2 for c in self.bs.cell_voltage]
            bs.present = True
            #self.log_info = "Battery state %, %".format(self.voltage, self.current)
            
        else:
            self.ina = self.setup_ina219(self.adress)
        
        pub.publish(bs)
#     def callback(self, msg): 
#         if msg.voltage < 6.4:
#             self.log_info = "Undervoltage"
#         elif msg.voltage == 0:
#             self.log_info = "Battery is absent"
#         else:
#             self.log_info = "Normal"
#         rospy.sleep(1)

        
    def start(self):
        while not rospy.is_shutdown():
            self.battery_talker(self.pub, self.bs)         
            self.battery_talker(self.msg_pub, self.msg.Battery)  
            #rospy.loginfo(self.log_info)
            self.seq += 1
            self.loop_rate.sleep()
            
        
if __name__ == '__main__':
    rospy.init_node('battery_state_tracker', anonymous = True)   
    bs = battery_state_proc(0x41)
    bs.start()
