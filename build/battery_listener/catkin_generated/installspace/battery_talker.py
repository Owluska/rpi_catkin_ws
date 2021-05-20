#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from library.rpi_telemetry import mb_telemetry



def talker():
    pub = rospy.Publisher("battery_voltage", Float32, queue_size = 10)
    rospy.init_node('battery_talker', anonymous = True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        mb.telemetry()
        try:
            pub.publish(mb.motors_voltage)
            rospy.loginfo("Battery voltage %.3f", mb.motors_voltage)
        except Exception as e:
            template = "An exception of type {0} occured. Arguments:\n{1!r}"
            message = template.format(type(e).__name__, e.args)
            rospy.loginfo("Battery voltage %s", message)
            rospy.sleep(2)

        rate.sleep()
if __name__ == '__main__':
    try:
        mb = mb_telemetry()
        mb.init_all()
        talker()
    except rospy.ROSInterruptException:
        pass
