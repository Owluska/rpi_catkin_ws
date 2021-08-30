#! /usr/bin/env python3
import rospy
import sys
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
        self.I = 0.0
        self.out = 0.0
    
    def compute_error(self, set_point, measured_variable):
        self.e = set_point - measured_variable
    
    def compute_out(self, set_point, measured_variable, dt):
        self.compute_error(set_point, measured_variable)
        P = self.Kp * self.e 
        self.I += self.Ki * self.e * dt
        D = self.Kd * (self.e - self.out)/dt
        self.out = P + D + self.I


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

        self.moving = False
        self.rotating = False


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
                
    
    def backward_moving_with_angle_crrctn(self, velocity = 60, angle_error = 1, P = 2.0):
        da = 0
        if self.yaw == None:
            return da
        if (self.state_status == self.errors_dic['ob_center'] or self.state_status == self.errors_dic['ob_right'] 
                                                              or self.state_status == self.errors_dic['ob_left']):
            self.stop()
            return da

        if abs(self.yaw) > angle_error:
            da = ((self.yaw  - self.center_angle) - angle_error)  
        
        new_angle = self.CENTER + P * da
        
        if abs(self.rul_position - new_angle) < angle_error:
            return da
        
        self.turn(new_angle)
        self.move(velocity, forward = False)
        return da              
            
    def forward_moving_with_angle_crrctn(self, t_start, t_stop = 5, velocity = 60, angle_error = 1, P = 2.0):
        da = 0
        if self.yaw == None:
            return da
        if self.t - t_start >= t_stop:
            self.stop()
            return da
        
        if abs(self.yaw) > angle_error:
            da = ((self.yaw  - self.center_angle) - angle_error)  
        
        new_angle = self.CENTER - P * da
        
        if abs(self.rul_position - new_angle) < angle_error:
            return da
        
        self.turn(new_angle)
        self.move(velocity, forward = True)
        return da  

    
    def counter(self, t0, secs):
        if self.t - t0 >= secs:
            return True
        else:
            return False
    
    def loop(self):
        tt = time()
        i = 0
        self.stop()
        s = 1
        for i in range(5):
            if self.yaw == None:
                self.get_telemetry()
            else: 
                break
        if self.yaw == None:
            return
        self.center_angle = self.yaw
        print_if = int(1/self.dt)

        t0 = self.t
        t_stop = 15
        while not rospy.is_shutdown():
            try:
                self.dt = time() - tt
                tt = time()
                self.t += self.dt
                self.get_telemetry()
                self.state_computing()
                # err = self.backward_moving_with_angle_crrctn()
                err = self.forward_moving_with_angle_crrctn(t0, t_stop = t_stop)
                              
                if(not bool(rospy.get_param("moving_state", False))):
                    sleep(5)
                    t0 = self.t
                if i % print_if == 0:
                    try:
                        rospy.loginfo("t:{:.2f}[s] t0{:.2f} st:{:d} ob1:{:.2f}[m] ob2:{:.2f}[m] yaw:{:.2f}[deg] rul_pos:{:.2f}".format(
                                    self.t, self.state_status, self.range_value1, self.range_value2, self.yaw, self.rul_position, t0))
                    except Exception:
                        pass
                i += 1
            except Exception as e:
                    template = "An exception of type {0} occured. Arguments:\n{1!r}"
                    message = template.format(type(e).__name__, e.args)
                    rospy.loginfo(message)

        self.stop()
            
def my_hook():
    rospy.loginfo("Stop signal was received")

def main():
    #print('car_us_movement')
    rospy.init_node('car_us_movement', anonymous = True)
    
    US1 = str(rospy.get_param('~US1_topic', default="US1_data"))
    US2 = str(rospy.get_param('~US2_topic', default="US2_data"))
    toStop = bool(rospy.get_param('~stop_car', default="False"))

    m = us_mvmnt(US1, US2)
    if toStop:
        m.stop()
        rospy.on_shutdown(my_hook)
        sys.exit(0)
        return
    else:
        m.loop()


if __name__ == '__main__':
    main()   