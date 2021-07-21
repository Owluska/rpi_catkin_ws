#! /usr/bin/env python3
import rospy

# from sensor_msgs.msg import Range, BatteryState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
from library.motors import PCA9685, car_movement_PCA9685
from rpicar.msg import telemetry
# from message_filters import TimeSynchronizer, Subscriber

from time import sleep
class us_mvmnt():
    min_voltage = 6.5
    
    def __init__(self, US1_topic, US2_topic):
        #self.pca = PCA9685(0x41)
        self.car = car_movement_PCA9685(PCA9685(0x41))
        
        self.CENTER = self.car.CENTER_DEGREE
        self.RIGHT = self.car.MIN_DEGREE
        self.LEFT = self.car.MAX_DEGREE

        self.MAX_SPEED = 100
        self.SPEED = int(self.MAX_SPEED * 0.75)


        self.telem_msg = telemetry()
        self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.callback)
        
        self.state_status = ""
        self.loop_rate = rospy.Rate(0.1)
        
        self.battery_voltage = 0.0
        self.range_value1 = 0.0
        self.range_value2 = 0.0
        self.min_range = 0.02

        self.vo_pose = [0, 0, 0]
        self.vo_quat = [0, 0, 0, 1]
        
        # self.point = [0, 0, 0]
        # self.quaternion = [0, 0, 0, 1]

        self.loop_rate= rospy.Rate(1)
        self.time_secs = rospy.get_time()
        #self.car_init()
    

    
    def callback(self, msg):       
        self.range_value1 = msg.US1.range
        self.range_value2 = msg.US2.range
        self.vo_pose = msg.VO.pose.pose.position
        self.vo_quat = msg.VO.pose.pose.orientation

    
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
            # self.car.turn(self.CENTER)
            # self.car.move_backward(self.SPEED)  
            sleep(1)
            self.time_secs -= rospy.get_time()
            rospy.loginfo("{:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(self.time_secs, self.range_value1, self.range_value2, self.vo_pose, self.vo_quat))

        self.car.stop_all()
        self.car.turn(self.CENTER)
            
        
def main():
    #print('car_us_movement')
    rospy.init_node('car_us_movement', anonymous = True)
    
    US1 = str(rospy.get_param('~US1_topic', default="US1_data"))
    US2 = str(rospy.get_param('~US2_topic', default="US2_data"))
    #print(US1, US2)
    m = us_mvmnt(US1, US2)
    m.loop()


if __name__ == '__main__':
    main()   