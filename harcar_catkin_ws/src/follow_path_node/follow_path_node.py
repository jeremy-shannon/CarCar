#!/usr/bin/env python
import rospy
import tf
from harcar_msgs.msg import CarControl
#from harcar_msgs.msg import Path
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from math import atan2, sqrt, pi, cos, sin, tan
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler, euler_from_quaternion

CONSTANT_SPEED = 1.25    # m/s
STEER_DAMPENING = 1.5   # unitless... 
WHEEL_BASE = 0.325      # meters
TURN_RATE_TOLERANCE = 0.001     # radians - tolerance for approximating straight-line motion

def dist(x1, y1, x2, y2):
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

class follow_path_node:
    def __init__(self):
        rospy.init_node('follow_path_node', anonymous=True)

        self.waypoints = None
        self.curXPos, self.curYPos, self.prevXPos, self.prevYPos = (None, None, None, None)
        self.heading, self.speed, self.steer_angle = (None, None, None)
        self.currentWaypointIndex = 0 

        self.lat0, self.lon0 = (42.51300695,-83.44528012)  # IF YOU EVER GO OUTSIDE THE HARMAN CABOT NORTH VICINITY, CHANGE THIS!!!!
        self.m_per_lat = (111132.954 - 559.822 * cos(2 * self.lat0) + 1.175 * cos(4 * self.lat0))
        self.m_per_lon = (311132.954 * cos(self.lon0))

        self.car_control_pub = rospy.Publisher("/car_control", CarControl, queue_size=1)
        self.rtk_pose = rospy.Publisher("/rtk_pose", PoseStamped, queue_size=1)
        self.car_control_vis_pub = rospy.Publisher("/car_control_vis", PoseStamped, queue_size=1)
        self.imu_pose_pub = rospy.Publisher("/imu_pose", PoseStamped, queue_size=1)
        self.car_state_pub = rospy.Publisher("/car_state_rviz", Odometry, queue_size=1)
        self.projected_path_pub = rospy.Publisher("/projected_path", Path, queue_size=1)
       
        self.car_pose_tf_br = tf.TransformBroadcaster()

        rospy.Subscriber("/waypoint_navmsg_path", Path, self.waypoints_cb)
        rospy.Subscriber("/tcpfix", NavSatFix, self.rtk_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/car_state", CarControl, self.car_state_cb)

        rospy.spin()
        
    def waypoints_cb(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.waypoints = data.poses

    def car_state_cb(self, data):
        self.steer_angle = data.steer_angle
        self.speed = data.speed
        rospy.loginfo("Steer: " + str(self.steer_angle) + ", speed: " + str(self.speed))

        # publish an Odometry message for Rviz representing the current car speed/steer
        if self.curXPos is not None and self.curYPos is not None and self.heading is not None:
            car_state_msg = Odometry()
            car_state_msg.header.stamp = rospy.get_rostime()
            car_state_msg.header.frame_id = "/car_rtk"
            #car_state_msg.header.frame_id = "/world"
            #car_state_msg.pose.pose.position.x = self.curXPos
            #car_state_msg.pose.pose.position.y = self.curYPos
            quat = quaternion_from_euler(0, 0, self.steer_angle)
            car_state_msg.pose.pose.orientation.x = quat[0]
            car_state_msg.pose.pose.orientation.y = quat[1]
            car_state_msg.pose.pose.orientation.z = quat[2]
            car_state_msg.pose.pose.orientation.w = quat[3]
            car_state_msg.twist.twist.linear.x = self.speed
            self.car_state_pub.publish(car_state_msg)

            # also publish a predicted path message
            predicted_path_msg = Path()
            predicted_path_msg.header.stamp = rospy.get_rostime()
            predicted_path_msg.header.frame_id = "/car_rtk"
            for t in range(0, 10):
                path_point = PoseStamped()
                path_point.header.stamp = rospy.get_rostime()
                path_point.header.frame_id = "/car_rtk"
                # bicycle motion model 
                distance = self.speed * float(t)
                length = WHEEL_BASE
                d_yaw = -1.0 * tan(self.steer_angle) * distance / length
                if abs(d_yaw) < TURN_RATE_TOLERANCE:
                    # approximate straight line motion
                    path_point.pose.position.x = distance
                else: 
                    # bicycle motion model
                    radius = distance / d_yaw
                    cx = 0
                    cy = radius
                    new_yaw = d_yaw % (2.0 * pi)
                    path_point.pose.position.x = cx + (sin(new_yaw) * radius)
                    path_point.pose.position.y = cy - (cos(new_yaw) * radius)
                predicted_path_msg.poses.append(path_point)
            self.projected_path_pub.publish(predicted_path_msg)

    def imu_cb(self, data):
        xPos, yPos = (0,0)
        if self.curXPos is not None:
            xPos = self.curXPos
        if self.curYPos is not None:
            yPos = self.curYPos
        xOri,yOri,zOri,wOri = (data.orientation.x, data.orientation.y,
                               data.orientation.z, data.orientation.w)
        imuMsg = PoseStamped()
        imuMsg.header.stamp = rospy.get_rostime()
        imuMsg.header.frame_id = "/world"
        imuMsg.pose.position.x = xPos
        imuMsg.pose.position.y = yPos
        imuMsg.pose.orientation.x = xOri
        imuMsg.pose.orientation.y = yOri
        imuMsg.pose.orientation.z = zOri
        imuMsg.pose.orientation.w = wOri
        self.imu_pose_pub.publish(imuMsg)
        _,_,self.heading = euler_from_quaternion((xOri,yOri,zOri,wOri))
        self.car_pose_tf_br.sendTransform((xPos, yPos, 0), 
                                          (xOri, yOri, zOri, wOri),
                                          rospy.Time.now(),
                                          "car_rtk", "world")

    def rtk_cb(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        control_msg = CarControl()
        control_vis_msg = PoseStamped()

        self.prevXPos = self.curXPos
        self.prevYPos = self.curYPos
        self.curXPos = (data.longitude - self.lon0) * self.m_per_lon
        self.curYPos = (data.latitude - self.lat0) * self.m_per_lat

        if self.waypoints is None:
            return

        if (self.curXPos and self.curYPos and self.prevXPos and self.prevYPos):
            xDiff, yDiff = (self.curXPos - self.prevXPos, self.curYPos - self.prevYPos)
            RTKHeading = atan2(yDiff, xDiff)
            quat = quaternion_from_euler(0, 0, RTKHeading)
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
            control_vis_msg = poseMsg  # copy over, we'll change the orientation later
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
        if distToCurrentWaypoint < 1.50:
            #rospy.loginfo("*****************Waypoint ", self.currentWaypointIndex," reached!********************")
            self.currentWaypointIndex += 1
            return

        xDiffToWaypoint, yDiffToWaypoint = (waypointX - self.curXPos, waypointY - self.curYPos)
        headingToWaypoint = atan2(yDiffToWaypoint, xDiffToWaypoint)

        # headingToWaypoint determines the car control message
        quat = quaternion_from_euler(0, 0, headingToWaypoint)
        control_vis_msg.pose.orientation.x = quat[0]
        control_vis_msg.pose.orientation.y = quat[1]
        control_vis_msg.pose.orientation.z = quat[2]
        control_vis_msg.pose.orientation.w = quat[3]
        self.car_control_vis_pub.publish(control_vis_msg)

        headingDiff = self.heading - headingToWaypoint
        if headingDiff < -pi:
            headingDiff += 2*pi
        if headingDiff > pi:
            headingDiff -= 2*pi
        steerValue = headingDiff / (STEER_DAMPENING * pi)    ######### FINE TUNE STEERING DAMPENING HERE ##############
        control_msg.steer_angle = steerValue
        control_msg.speed = CONSTANT_SPEED
        self.car_control_pub.publish(control_msg)

if __name__ == '__main__':
    try:
        follow_path_node()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start follow path node.')
