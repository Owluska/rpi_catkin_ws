#! /usr/bin/env python3

import wiringpi
import roslib
import rospy
from sensor_msgs.msg import Range, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from library.motors import PCA9685, car_movement_PCA9685
import message_filters

class us_mvmnt():
    min_voltage = 6.5
    # RUL = 12
    # MOTOR_A = 6
    # MOTOR_B = 26
    
    # RUL_CENTER = 150
    # RUL_RIGHT = 200
    # RUL_LEFT = 100
    
    def __init__(self, US1_topic, US2_topic):
        self.pca = PCA9685(0x41)
        self.car = car_movement_PCA9685(self.pca)
        
        self.CENTER = self.car.CENTER_DEGREE
        self.RIGHT = self.car.MIN_DEGREE
        self.LEFT = self.car.MAX_DEGREE

        self.MAX_SPEED = self.car.MAX_SPEED
        self.SPEED = int(self.MAX_SPEED * 0.75)

        self.us1_sub = rospy.Subscriber(US1_topic, Range, self.us1_callback, queue_size = 10)
        self.us2_sub = rospy.Subscriber(US2_topic, Range, self.us2_callback, queue_size = 10)
        self.bat_sub = rospy.Subscriber("battery_state", BatteryState, self.bat_callback, queue_size = 10)
        #self.odom_sub = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.pos_callback, queue_size = 10)
        self.odom_sub = rospy.Subscriber("vo", Odometry, self.odom_callback, queue_size = 10)
        self.topics = [self.us1_sub, self.us2_sub, self.bat_sub, self.odom_sub]
        self.ts = message_filters.TimeSynchronizer(self.topics, 10)
        self.state_status = ""
        self.loop_rate = rospy.Rate(0.1)
        
        self.battery_voltage = 0.0
        self.range_value1 = 0.0
        self.range_value2 = 0.0
        self.min_range = 0.02
        self.point = [0, 0, 0]
        self.quaternion = [0, 0, 0, 1]

        self.loop_rate= rospy.Rate(1)
        #self.car_init()
    
    def us1_callback(self, msg):
        print("US1")
        self.range_value1 = msg.range
        if msg.range <= msg.min_range:
            self.state_status = "Obastcale"
    
    def us2_callback(self, msg):
        print("US2")
        self.range_value2 = msg.range
        if msg.range <= msg.min_range:
            self.state_status = "Obastcale"
            
    def bat_callback(self, msg):
        self.battery_voltage = msg.voltage
        if msg.voltage <= self.min_voltage:
            self.state_status = "Undervoltage"
    
    def pos_callback(self, msg):
        print("pos")
        self.point[0] = msg.pose.pose.position.x
        self.point[1] = msg.pose.pose.position.y
        self.point[2] = msg.pose.pose.position.z
        
        self.quaternion[3] = msg.pose.pose.orientation.w

    def odom_callback(self, msg):
        print("Got odom")       
    
    def random_movement(self):       
        if self.state_status == "Undervoltage":
            self.car.stop_all()
            self.car.turn(self.CENTER)
            rospy.loginfo(self.state_status + ", so ending node..")
            rospy.on_shutdown(self.state_status)
            return -1
        
        elif self.state_status == "Obastcale":
            if self.range_value1 <= self.min_range:
                self.car.turn(self.RIGHT)
                self.car.move_forward(self.MAX_SPEED)
            elif self.range_value2 <= self.min_range:
                self.car.turn(self.LEFT)
                self.car.move_forward(self.MAX_SPEED)
        else:
                self.car.turn(self.CENTER)
                self.car.move_backward(self.SPEED)            
    
    def loop(self):
        while not rospy.is_shutdown():
            self.random_movement() 
            self.loop_rate.sleep()
            print("{} s, {} m, ".format(rospy.Time, self.point, self.quaternion))
            self.loop_rate.sleep()
        self.car.stop_all()
        self.car.turn(self.CENTER)
            
        
def main():
    rospy.init_node('car_us_movement', anonymous = True)
    US1 = str(rospy.get_param('~US1_topic', default="US1_data"))
    US2 = str(rospy.get_param('~US2_topic', default="US2_data"))
    print(US1, US2)
    m = us_mvmnt(US1, US2)
    #m.loop()


if __name__ == '__main__':
    main()   