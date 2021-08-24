#! /usr/bin/env python3
import rospy

# from sensor_msgs.msg import Range, BatteryState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
from drivers.motors import PCA9685, car_movement_PCA9685
from rpicar.msg import telemetry

# from message_filters import TimeSynchronizer, Subscriber
from time import sleep, time

# class PID:
#     def __init__(self):
#         self.I = 0.0
#         self.P = 0.0
#         self.D = 0.0

#         self.summ_err = 0.0
#         self.diff_err = 0.0
#         self.dt = 0.0
class us_mvmnt():
    min_voltage = 6.5
    min_range = 0.1
    def __init__(self, US1_topic, US2_topic):
        #self.pca = PCA9685(0x41)
        self.car = car_movement_PCA9685(PCA9685(0x41))
        
        self.CENTER = self.car.CENTER_DEGREE
        self.RIGHT = self.car.MIN_DEGREE
        self.LEFT = self.car.MAX_DEGREE

        self.MAX_SPEED = 100
        self.SPEED = int(self.MAX_SPEED * 0.6)


        # self.telem_msg = telemetry()
        # self.telem_sub = rospy.Subscriber("telemetry_chatter", telemetry, self.callback)
                
        
        self.battery_voltage = 0.0
        self.range_value1 = None
        self.range_value2 = None
        
        # self.min_range = 0.02

        self.px = None
        self.py = None
        self.pz = None

        self.roll = None
        self.pitch = None
        self.yaw = None
        
        self.dt = 0.01
        self.t = 0.00
        
        # self.point = [0, 0, 0]
        # self.quaternion = [0, 0, 0, 1]

        self.rate = rospy.Rate(0.01)
        # self.time_secs = 0
        # self.dt = rospy.get_time()

        self.errors_dic = {"ob_left": 1, "ob_right":2, "ob_center":3, "undervoltage":4, "none":0}
        self.state_status = 0
        self.center_angle = 0.0
        #self.car_init()
    

    
    # def callback(self, msg):       
    #     self.range_value1 = msg.US1.range
    #     self.range_value2 = msg.US2.range

    #     self.vo_px = msg.VO.pose.pose.position.x
    #     self.vo_py = msg.VO.pose.pose.position.y
    #     self.vo_pz = msg.VO.pose.pose.position.z

    #     self.vo_qx = msg.VO.pose.pose.orientation.x
    #     self.vo_qy = msg.VO.pose.pose.orientation.y
    #     self.vo_qz = msg.VO.pose.pose.orientation.z
    #     self.vo_qw = msg.VO.pose.pose.orientation.w
    #     # self.vo_pose = msg.VO.pose.pose.position
    #     # self.vo_quat = msg.VO.pose.pose.orientation

    def get_telemetry(self):
        try:
            self.range_value1 = rospy.get_param('US1_data')
            self.range_value2 = rospy.get_param('US2_data')
            

            self.roll = rospy.get_param('orientation')['roll']
            self.pitch = rospy.get_param('orientation')['pitch']
            self.yaw = rospy.get_param('orientation')['yaw']

            # self.px = rospy.get_param('position')['x']
            # self.py = rospy.get_param('position')['y']
            # self.pz = rospy.get_param('position')['z']
        
        except Exception as e:
                template = "An exception of type {0} occured. Arguments:\n{1!r}"
                message = template.format(type(e).__name__, e.args)
                rospy.loginfo(message)
    
    def state_computing(self):
        if self.range_value2 <= self.min_range:
            key = "ob_left"
            self.state_status = self.errors_dic[key]
        
        elif self.range_value1 <= self.min_range:
            key = "ob_right"
            self.state_status = self.errors_dic[key]
       
        elif self.range_value1 <= self.min_range and self.range_value2 <= self.min_range:
            key = "ob_center"
            self.state_status = self.errors_dic[key]
        
        else:
            key = "none"
            self.state_status = self.errors_dic[key]
        # data = str(self.errors_dic[key])
        # rospy.loginfo(data)

    def turn_on_angle(self, angle = 90):
        self.get_telemetry()
        if self.yaw == None or angle == 0:
            return
        
        if abs(angle) > 180:
            rospy.loginfo("Get wrong parameter value! Please, set angle in [-180, 180] range.") 
            return
               
        if angle < 0:
            sign = -1
            self.car.turn(self.RIGHT)       
        else:
            sign = 1
            self.car.turn(self.LEFT)
        
        target_angle = angle - sign * self.yaw
        if abs(target_angle) > 180:
            target_angle -= sign * 180 
        rospy.loginfo("Start: yaw:{:.2f}[deg] target:{:.2f}[deg], get: {:.2f}".format(self.yaw, target_angle, angle)) 
        
        while(not rospy.is_shutdown()):
            try:         
                self.get_telemetry() 
                if sign > 0 and self.yaw > target_angle:
                    return 
                elif sign < 0 and self.yaw < target_angle:
                    return
                elif self.state_status == self.errors_dic['none']:
                    return
                self.car.move_forward(self.SPEED)
                rospy.loginfo("t:{:.2f}[s] yaw:{:.2f}[deg] target:{:.2f}[deg]".format(self.t, self.yaw, target_angle))
            
            except Exception:
                break
        
    
    
    def backward_moving_n_correction(self, velocity = 50, backward = True, angle_error = 2, P = 1.0):
        da = 0
        if self.yaw == None:
            return da
        if (self.state_status == self.errors_dic['ob_center'] or self.state_status == self.errors_dic['ob_right'] 
                                                              or self.state_status == self.errors_dic['ob_left']):
            self.stop_car()
            return da
        self.car.turn(self.CENTER)
        if abs(self.yaw) > angle_error:
            da = ((self.yaw  - self.center_angle) - angle_error)  
        
        self.car.turn(self.CENTER + P * da)   
        self.car.move_backward(velocity)
        self.rate.sleep()
        return da              
            

    def random_movement(self):
        rospy.set_param("moving_state", True)
        angle = 90
        rospy.set_param("rotating_state", False)
        
        if self.state_status == self.errors_dic['undervoltage']:
            self.car.stop_all()
            self.car.turn(self.CENTER)
            rospy.set_param("moving_state", False)  
            rospy.loginfo(self.state_status + ", so ending node..")
            rospy.on_shutdown(self.state_status)
            return -1
        
        
        elif self.state_status == self.errors_dic['ob_left']:
                self.turn_on_angle(angle)
                rospy.set_param("rotating_state", True)  
                self.car.move_forward(self.MAX_SPEED)
        
        elif self.state_status == self.errors_dic['ob_right']:
                self.turn_on_angle(-angle)
                rospy.set_param("rotating_state", True)  
                self.car.move_forward(self.MAX_SPEED)
        
        elif self.state_status == self.errors_dic['ob_center']:
                self.car.turn(self.CENTER)
                self.car.move_forward(self.SPEED)
        else:
                self.car.turn(self.CENTER)
                self.car.move_backward(self.SPEED) 
                        
    
    def stop_car(self):
        self.car.stop_all()
        rospy.set_param("moving_state", False)
        self.car.turn(self.CENTER)
        rospy.set_param("rotating_state", False)
    
    def loop(self):
        tt = time()
        i = 0
        self.stop_car()
        s = 1
        for i in range(20):
            if self.yaw == None:
                self.get_telemetry()
            else: 
                break
        if self.yaw == None:
            return
        self.center_angle = self.yaw
        print_if = int(1/self.dt)
        # self.car.turn(self.CENTER)
        # sleep(5.0)
        while not rospy.is_shutdown():
            try:
                self.dt = time() - tt
                tt = time()
                self.t += self.dt
                self.get_telemetry()
                self.state_computing()
                #err = self.backward_moving_n_correction()
                #self.car.move_backward(self.SPEED) 
                #self.turn_on_angle(angle = s * 90)
                #self.random_movement()
                if i % print_if == 0:
                    try:
                        rospy.loginfo("t:{:.2f}[s] st:{:d} ob1:{:.2f}[m] ob2:{:.2f}[m] yaw:{:.2f}[deg] err:{:.2f}[deg]".format(
                                    self.t, self.state_status, self.range_value1, self.range_value2, self.yaw, err))
                    except Exception:
                        pass

                # self.dt = rospy.get_time()
                #self.car.stop_all()
                #sleep(5)
                i += 1
                # s *= -1
                #self.loop_rate.sleep()
            except Exception:
                break

        self.stop_car()
            
        
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