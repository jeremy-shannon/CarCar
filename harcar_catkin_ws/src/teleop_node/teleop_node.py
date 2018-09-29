#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from harcar_msgs.msg import CarControl
from math import pi

# JOYSTICK RANGES:
# left stick (axes[0]): full left = 1.0, full right = -1.0
# right trigger (axes[5]): nominal = 1.0, fully pressed = -1.0
# A button (buttons[0]): [0,1]

class teleop_node:
    def __init__(self):
        rospy.init_node('teleop_node', anonymous=True)
        rospy.loginfo(rospy.get_caller_id() + "INITIALIZED teleop_node")

        self.car_cmd_pub = rospy.Publisher("/car_control", CarControl, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        
        rospy.spin()

    def joy_cb(self, data):
        left_stick = data.axes[0]
        right_trigger = data.axes[5]
        a_button = data.buttons[0]

        steer = left_stick * pi / 6.0
        speed = -0.5 * (right_trigger - 1.0)

        car_cmd_msg = CarControl()
        car_cmd_msg.steer_angle = steer
        car_cmd_msg.speed = speed
        self.car_cmd_pub.publish(car_cmd_msg)

if __name__ == '__main__':
    try:
        teleop_node()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start teleop node')
