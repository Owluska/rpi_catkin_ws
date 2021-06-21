#! /usr/bin/env python3
from sensor_msgs.msg import Range, BatteryState
import wiringpi

class us_mvmnt():
    min_voltage = 6.5
    RUL = 12
    MOTOR_A = 6
    MOTOR_B = 26
    
    RUL_CENTER = 150
    RUL_RIGHT = 200
    RUL_LEFT = 100
    
    def __init__(self, US1_topic, US2_topic):
        self.us1_sub = rospy.Subscriber(US1_topic, Range, self.us1_callback, queue_size = 10)
        self.us2_sub = rospy.Subscriber(US2_topic, Range, self.us2_callback, queue_size = 10)
        self.bat_sub = rospy.Subscriber("battery_state", BatteryState, self.bat_callback, queue_size = 10)
        self.state_status = ""
        self.loop_rate = rospy.Rate(0.1)
        
        self.battery_voltage = 0.0
        self.range_value1 = 0.0
        self.range_value2 = 0.0
        self.min_range = 0.02
        self.car_init()
    
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
    
    def car_init(self):
        wiringpi.wiringPiSetupGpio()
    
        wiringpi.pinMode(self.MOTOR_A, wiringpi.GPIO.OUTPUT)
        wiringpi.pinMode(self.MOTOR_B, wiringpi.GPIO.OUTPUT)

        wiringpi.pullUpDnControl(self.MOTOR_A, wiringpi.GPIO.PUD_UP)
        wiringpi.pullUpDnControl(self.MOTOR_B, wiringpi.GPIO.PUD_UP)
    
        wiringpi.pinMode(self.RUL, wiringpi.GPIO.PWM_OUTPUT)
    
        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
        wiringpi.pwmSetClock(192)
        wiringpi.pwmSetRange(2000)
    
    def stop(self):
        wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.HIGH)
        wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.HIGH)

        
    def move_forward(self):
        wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.LOW)
        wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.HIGH)
 
        
    def move_backward(self):
        wiringpi.digitalWrite(self.MOTOR_A, wiringpi.GPIO.HIGH)
        wiringpi.digitalWrite(self.MOTOR_B, wiringpi.GPIO.LOW)

        
    def turn_right(self):
        wiringpi.pwmWrite(self.RUL, self.RUL_RIGHT)
        
    def turn_left(self):
        wiringpi.pwmWrite(self.RUL, self.RUL_LEFT)
    
    def turn_center(self):
        wiringpi.pwmWrite(self.RUL, self.RUL_CENTER)
        
    def random_movement():       
        if self.state_status == "Undervoltage":
            self.stop()
            rospy.loginfo(self.state_status + ", so ending node..")
            rospy.on_shutdown(self.state_status)
            return -1
        
        elif self.state_status == "Obastcale":
            if self.range_value1 <= self.min_range:
                self.turn_right()
                self.move_forward()
            elif
                self.turn_left()
                self.move_forward()
        else:
                self.turn_center()
                self.move_backward()            
    
    def loop(self):
        while not rospy.is_shutdown():
            self.random_movement() 
            self.loop_rate.sleep()
            
        
def main():
    rospy.init_node('car_us_movement', anonymous = True)
    US1 = str(rospy.get_param('~US1_topic', default="US1_data"))
    US2 = str(rospy.get_param('~US2_topic', default="US2_data"))
    m = us_mvmnt(US1, US2)
    m.loop()


if __name__ == '__main__':
    main()   