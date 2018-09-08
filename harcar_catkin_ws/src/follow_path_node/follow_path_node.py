#!/usr/bin/env python
import rospy
from harcar_msgs.msg import CarControl
from harcar_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from math import atan2, sqrt, pi

CONSTANT_SPEED = 1.0    # m/s

def dist(x1, y1, x2, y2):
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

class follow_path_node:
    def __init__(self):
        rospy.init_node('follow_path_node', anonymous=True)

        rospy.Subscriber("/waypoint_path", Path, self.waypoints_cb)
        rospy.Subscriber("/tcpfix", NavSatFix, self.rtk_cb)
        self.car_control_pub = rospy.Publisher("/car_control", CarControl, queue_size=1)

        self.waypoints = None
        self.curXPos, self.curYPos, self.prevXPos, self.prevYPos = (None, None, None, None)
        self.heading, self.speed = (None, None)
        self.currentWaypointIndex = 0 

        rospy.spin()
        
    def waypoints_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.waypoints = data.path

    def rtk_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        control_msg = CarControl()

        self.prevXPos = self.curXPos
        self.prevYPos = self.curYPos
        self.curXPos = data.longitude
        self.curYPos = data.latitude

        if self.waypoints is None:
            return

        if (self.curXPos and self.curYPos and self.prevXPos and self.prevYPos):
            xDiff, yDiff = (self.curXPos - self.prevXPos, self.curYPos - self.prevYPos)
            self.heading = atan2(yDiff, xDiff)
        else:
            # not enough info yet - need two rtk readings
            return
        
        if self.currentWaypointIndex == len(self.waypoints):
            # reached final waypoint - stop
            control_msg.steer_angle = 0
            control_msg.speed = 0
            self.car_control_pub.publish(control_msg)
            return

        waypointX = self.waypoints[self.currentWaypointIndex].position.x
        waypointY = self.waypoints[self.currentWaypointIndex].position.y
        distToCurrentWaypoint = dist(self.curXPos, self.curYPos, waypointX, waypointY)
        if distToCurrentWaypoint < 0.000038:
            #rospy.loginfo("*****************Waypoint ", self.currentWaypointIndex," reached!********************")
            self.currentWaypointIndex += 1
            return

        xDiffToWaypoint, yDiffToWaypoint = (waypointX - self.curXPos, waypointY - self.curYPos)
        headingToWaypoint = atan2(yDiffToWaypoint, xDiffToWaypoint)
        headingDiff = self.heading - headingToWaypoint
        if headingDiff < -pi:
            headingDiff += 2*pi
        if headingDiff > pi:
            headingDiff -= 2*pi
        steerValue = headingDiff/(pi/1.25)
        control_msg.steer_angle = steerValue
        control_msg.speed = CONSTANT_SPEED
        self.car_control_pub.publish(control_msg)

if __name__ == '__main__':
    try:
        follow_path_node()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start follow path node.')
