#! /usr/bin/env python3
import rospy

# from sensor_msgs.msg import Range, BatteryState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
from drivers.motors import PCA9685, car_movement_PCA9685
from rpicar.msg import telemetry

# from message_filters import TimeSynchronizer, Subscriber
from time import sleep, time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd

        self.e = 0.0
        self.out = 0.0
    
    def compute_error(self, set_point, measured_variable):
        self.e = set_point - measured_variable
    
    def compute_out(self, set_point, measured_variable, dt):
        self.compute_error(set_point, measured_variable)
        P = self.Kp * self.e 
        I += self.Ki * self.e * dt
        D = self.Kd * (self.e - self.out)/dt
        self.out = self.e


class us_mvmnt():
    #this will be common for all instances of US_MVMNT
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
           
        
        self.battery_voltage = 0.0
        self.range_value1 = None
        self.range_value2 = None
        

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
        self.rul_position = self.CENTER
        #self.car_init()
    
    def stop(self):
        self.car.stop_all()
        rospy.set_param("moving_state", False)
        self.car.turn(self.CENTER)
        rospy.set_param("rotating_state", False)
    
    def move(self, velocity, forward = True):
        if forward:
            self.car.move_forward(velocity)
        else:
            self.car.move_backward(velocity)
        rospy.set_param("moving_state", True)

    def turn(self, angle):
        self.rul_position = self.car.turn(angle)
        if angle != self.CENTER:
              rospy.set_param("rotating_state", True)  

    def get_telemetry(self):
        try:
            self.range_value1 = rospy.get_param('US1_data')
            self.range_value2 = rospy.get_param('US2_data')
            

            self.roll = rospy.get_param('orientation')['roll']
            self.pitch = rospy.get_param('orientation')['pitch']
            self.yaw = rospy.get_param('orientation')['yaw']
        
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
        
        if sign > 0 and self.yaw > target_angle:
            return 
        elif sign < 0 and self.yaw < target_angle:
            return
        elif self.state_status == self.errors_dic['none']:
            return
        self.car.move_forward(self.SPEED)
                
    
    def backward_moving_n_correction(self, velocity = 60, backward = True, angle_error = 1, P = 2.0):
        da = 0
        if self.yaw == None:
            return da
        if (self.state_status == self.errors_dic['ob_center'] or self.state_status == self.errors_dic['ob_right'] 
                                                              or self.state_status == self.errors_dic['ob_left']):
            self.stop_car()
            return da

        if abs(self.yaw) > angle_error:
            da = ((self.yaw  - self.center_angle) - angle_error)  
        
        new_angle = self.CENTER + P * da
        
        if abs(self.rul_position - new_angle) < angle_error:
            return da
        
        self.turn(new_angle)
        self.move(velocity, forward = False)
        # if da != 0:
        #     #servo vel ~ 4ms/deg
        #     t = int(abs(P * da * 4))
        #     for i in range(int(t)):
        #         try:
        #             sleep(.001)
        #         except Exception:
        #             break
        #self.rate.sleep()
        return da              
            

    def random_movement(self):
        rospy.set_param("moving_state", True)
        angle = 90
        rospy.set_param("rotating_state", False)
        
        if self.state_status == self.errors_dic['undervoltage']:
            self.stop()
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
                        
    
 
    
    def loop(self):
        tt = time()
        i = 0
        self.stop()
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
        while not rospy.is_shutdown():
            try:
                self.dt = time() - tt
                tt = time()
                self.t += self.dt
                self.get_telemetry()
                self.state_computing()
                err = self.backward_moving_n_correction()
                #self.turn_on_angle(angle = s * 90)
                #self.random_movement()
                if i % print_if == 0:
                    try:
                        rospy.loginfo("t:{:.2f}[s] st:{:d} ob1:{:.2f}[m] ob2:{:.2f}[m] yaw:{:.2f}[deg] err:{:.2f}[deg] rul_pos:{:.2f}".format(
                                    self.t, self.state_status, self.range_value1, self.range_value2, self.yaw, err, self.rul_position))
                    except Exception:
                        pass
                i += 1
            except Exception as e:
                    template = "An exception of type {0} occured. Arguments:\n{1!r}"
                    message = template.format(type(e).__name__, e.args)
                    rospy.loginfo(message)

        self.stop()
            
        
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