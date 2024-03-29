#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from harcar_msgs.msg import CarControl
from math import pi

# JOYSTICK RANGES:
# left stick (axes[0]): full left = 1.0, full right = -1.0
# left/right trigger (axes[2/5]): nominal = 1.0, fully pressed = -1.0
# A button (buttons[0]): [0,1]

class teleop_node:
    def __init__(self):
        rospy.init_node('teleop_node', anonymous=True)
        rospy.loginfo(rospy.get_caller_id() + " INITIALIZED teleop_node")

        self.car_cmd_pub = rospy.Publisher("/car_control", CarControl, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        self.TURBO_MODE_ACTIVE = False
        self.speed_multiplier = -0.5
        
        rospy.spin()

    def joy_cb(self, data):
        left_stick = data.axes[0]
        left_trigger = data.axes[2]
        right_trigger = data.axes[5]
        a_button = data.buttons[0]

        #This should only execute when the A button is pressed. However, if a user holds the button the value will remain at one. So if the user is holding the A button and presses another button it will call the callback again and toggle the TURBO_MODE. Not good!
        if a_button == True:
            if self.TURBO_MODE_ACTIVE == True:
                self.TURBO_MODE_ACTIVE = False
                self.speed_multiplier = -0.5
            else:
                self.TURBO_MODE_ACTIVE = True
                self.speed_multiplier = -1.0

        steer = left_stick * pi / 6.0
        # forward_speed = -0.5 * (right_trigger - 1.0)
        # backward_speed = -0.5 * (left_trigger - 1.0)
        # speed = forward_speed - backward_speed
        # speed = self.speed_multiplier * (right_trigger - 1.0)
        forward_speed = self.speed_multiplier * (right_trigger - 1.0)
        backward_speed = -0.5 * (left_trigger - 1.0)
        speed = forward_speed - backward_speed

        car_cmd_msg = CarControl()
        car_cmd_msg.steer_angle = steer
        car_cmd_msg.speed = speed
        self.car_cmd_pub.publish(car_cmd_msg)

if __name__ == '__main__':
    try:
        teleop_node()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start teleop node')
