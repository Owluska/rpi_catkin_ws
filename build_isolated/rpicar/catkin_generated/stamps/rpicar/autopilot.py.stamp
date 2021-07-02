#! /usr/bin/env python3

import wiringpi
import roslib
import rospy
from sensor_msgs.msg import Range, BatteryState
from nav_msgs.msg import Odometry
from library.motors import PCA9685, car_movement_PCA9685

class us_mvmnt():
    min_voltage = 6.5
    # RUL = 12
    # MOTOR_A = 6
    # MOTOR_B = 26
    
    # RUL_CENTER = 150
    # RUL_RIGHT = 200
    # RUL_LEFT = 100
    
    def __init__(self, US1_topic, US2_topic):
        self.pca = PCA9685()
        self.car = car_movement_PCA9685(self.pca)
        
        self.CENTER = self.car.CENTER_DEGREE
        self.RIGHT = self.car.MIN_DEGREE
        self.LEFT = self.car.MAX_DEGREE

        self.MAX_SPEED = self.car.MAX_SPEED
        self.SPEED = int(self.MAX_SPEED * 0.75)

        self.us1_sub = rospy.Subscriber(US1_topic, Range, self.us1_callback, queue_size = 10)
        self.us2_sub = rospy.Subscriber(US2_topic, Range, self.us2_callback, queue_size = 10)
        self.bat_sub = rospy.Subscriber("battery_state", BatteryState, self.bat_callback, queue_size = 10)
        self.odom_sub = rospy.Subscriber("robot_pose_ekf/odom_combined", Odometry, self.pos_callback, queue_size = 10)
        self.state_status = ""
        self.loop_rate = rospy.Rate(0.1)
        
        self.battery_voltage = 0.0
        self.range_value1 = 0.0
        self.range_value2 = 0.0
        self.min_range = 0.02
        self.point = [0, 0, 0]
        self.quaternion = [0, 0, 0, 1]
        #self.car_init()
    
    def us1_callback(self, msg):
        self.range_value1 = msg.range
        if msg.range <= msg.min_range:
            self.state_status = "Obastcale"
    
    def us2_callback(self, msg):
        self.range_value2 = msg.range
        if msg.range <= msg.min_range:
            self.state_status = "Obastcale"
            
    def bat_callback(self, msg):
        self.battery_voltage = msg.voltage
        if msg.voltage <= self.min_voltage:
            self.state_status = "Undervoltage"
    def pos_callback(self, msg):
        self.point = msg.pose.pose.point
        self.quaternion = msg.pose.pose.quternion
    
    # def car_init(self):
    #     wiringpi.wiringPiSetupGpio()
    
    #     wiringpi.pinMode(self.MOTOR_A, wiringpi.GPIO.OUTPUT)
    #     wiringpi.pinMode(self.MOTOR_B, wiringpi.GPIO.OUTPUT)

    #     wiringpi.pullUpDnControl(self.MOTOR_A, wiringpi.GPIO.PUD_UP)
    #     wiringpi.pullUpDnControl(self.MOTOR_B, wiringpi.GPIO.PUD_UP)
    
    #     wiringpi.pinMode(self.RUL, wiringpi.GPIO.PWM_OUTPUT)
    
    #     wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
    #     wiringpi.pwmSetClock(192)
    #     wiringpi.pwmSetRange(2000)
    
    # def stop(self):
    #     wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.HIGH)
    #     wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.HIGH)

        
    # def move_forward(self):
    #     wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.LOW)
    #     wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.HIGH)
 
        
    # def move_backward(self):
    #     wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.HIGH)
    #     wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.LOW)

        
    # def turn_right(self):
    #     wiringpi.pwmWrite(self.RUL, self.RUL_RIGHT)
        
    # def turn_left(self):
    #     wiringpi.pwmWrite(self.RUL, self.RUL_LEFT)
    
    # def turn_center(self):
    #     wiringpi.pwmWrite(self.RUL, self.RUL_CENTER)
        
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
        self.car.stop_all()
        self.car.turn(self.CENTER)
            
        
def main():
    rospy.init_node('car_us_movement', anonymous = True)
    US1 = str(rospy.get_param('~US1_topic', default="US1_data"))
    US2 = str(rospy.get_param('~US2_topic', default="US2_data"))
    print(US1, US2)
    m = us_mvmnt(US1, US2)
    m.loop()


if __name__ == '__main__':
    main()   