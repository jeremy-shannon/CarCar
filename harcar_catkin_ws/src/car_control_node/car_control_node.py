#!/usr/bin/env python
import rospy
from HarCarThreaded import HarCar
from harcar_msgs.msg import CarControl

class car_control_node:
    def __init__(self):
        rospy.init_node('car_control_node', anonymous=True)
        rospy.loginfo(rospy.get_caller_id() + "INITED Car_Control_node")
        #create harcar object which handles m/s & rad. conversions into 
        #pwm and uses pwm library to input that info to pwm
        self.car = HarCar()

        state_pub = rospy.Publisher("/car_state", CarControl, queue_size=1)
        rospy.Subscriber("/car_control", CarControl, self.control_cb)#TODO Correct topic and message
        
        rate = rospy.Rate(10) # Hz. 

        while not rospy.is_shutdown():
            # do whatever you want here
            current_speed, current_steer = self.car.get_speed_and_steer()
            state_msg = CarControl()
            state_msg.speed = current_speed
            state_msg.steer_angle = current_steer
            state_pub.publish(state_msg)
            rate.sleep()
        
        self.car.shut_down()

    def control_cb(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.car.set_steer(data.steer_angle)#TODO Correct steer_angle
        self.car.set_speed(data.speed)#TODO Correct Speed

if __name__ == '__main__':
    try:
        car_control_node()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start car control node')
