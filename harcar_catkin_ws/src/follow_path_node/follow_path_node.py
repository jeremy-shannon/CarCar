#!/usr/bin/env python
import rospy
from harcar_msgs.msg import CarControl
#from harcar_msgs.msg import Path
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from math import atan2, sqrt, pi, cos
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler

CONSTANT_SPEED = 1.0    # m/s

def dist(x1, y1, x2, y2):
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

class follow_path_node:
    def __init__(self):
        rospy.init_node('follow_path_node', anonymous=True)

        rospy.Subscriber("/waypoint_navmsg_path", Path, self.waypoints_cb)
        rospy.Subscriber("/tcpfix", NavSatFix, self.rtk_cb)
        self.car_control_pub = rospy.Publisher("/car_control", CarControl, queue_size=1)
        self.rtk_pose = rospy.Publisher("/rtk_pose", PoseStamped, queue_size=1)

        self.waypoints = None
        self.curXPos, self.curYPos, self.prevXPos, self.prevYPos = (None, None, None, None)
        self.heading, self.speed = (None, None)
        self.currentWaypointIndex = 0 

        self.lat0, self.lon0 = (42.51300695,-83.44528012)  # IF YOU EVER GO OUTSIDE THE HARMAN CABOT NORTH VICINITY, CHANGE THIS!!!!
        self.m_per_lat = (111132.954 - 559.822 * cos(2 * self.lat0) + 1.175 * cos(4 * self.lat0))
        self.m_per_lon = (311132.954 * cos(self.lon0))

        rospy.spin()
        
    def waypoints_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.waypoints = data.poses

    def rtk_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        control_msg = CarControl()

        self.prevXPos = self.curXPos
        self.prevYPos = self.curYPos
        self.curXPos = (data.longitude - self.lon0) * self.m_per_lon
        self.curYPos = (data.latitude - self.lat0) * self.m_per_lat

        if self.waypoints is None:
            return

        if (self.curXPos and self.curYPos and self.prevXPos and self.prevYPos):
            xDiff, yDiff = (self.curXPos - self.prevXPos, self.curYPos - self.prevYPos)
            self.heading = atan2(yDiff, xDiff)
            quat = quaternion_from_euler(0, 0, self.heading)
            # publish PoseStamped message
            poseMsg = PoseStamped()
            poseMsg.header.stamp = rospy.get_rostime()
            poseMsg.header.frame_id = "/world"
            poseMsg.pose.position.x = self.curXPos
            poseMsg.pose.position.y = self.curYPos
            poseMsg.pose.orientation.x = quat[0]
            poseMsg.pose.orientation.y = quat[1]
            poseMsg.pose.orientation.z = quat[2]
            poseMsg.pose.orientation.w = quat[3]
            self.rtk_pose.publish(poseMsg)
        else:
            # not enough info yet - need two rtk readings
            return
        
        if self.currentWaypointIndex == len(self.waypoints):
            # reached final waypoint - stop
            control_msg.steer_angle = 0
            control_msg.speed = 0
            self.car_control_pub.publish(control_msg)
            return

        waypointX = self.waypoints[self.currentWaypointIndex].pose.position.x
        waypointY = self.waypoints[self.currentWaypointIndex].pose.position.y
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
