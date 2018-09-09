#!/usr/bin/env python
import csv
import rospy
import math
#from harcar_msgs.msg import Path
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def load_points():
    pub = rospy.Publisher('waypoint_navmsg_path', Path, queue_size=1)
    rospy.init_node('load_path', anonymous=True)
    rate = rospy.Rate(0.2) #Hz. Once every 5 seconds.
    waypoints = Path()
    lat0, lon0 = (42.51300695,-83.44528012)   # IF YOU EVER GO OUTSIDE THE HARMAN CABOT NORTH VICINITY, CHANGE THIS!!!!
    m_per_lat = (111132.954 - 559.822 * math.cos(2 * lat0) + 1.175 * math.cos(4 * lat0))
    m_per_lon = (311132.954 * math.cos(lon0))
    with open('/home/ubuntu/harcar_catkin_ws/src/load_path_node/Figure8_spaced30.csv') as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            waypoint = PoseStamped()
            waypoint.header.stamp = rospy.get_rostime()
            waypoint.header.frame_id = "/world"
            lon = float(row[1])
            lat = float(row[0])
            waypoint.pose.position.x = (lon - lon0) * m_per_lon
            waypoint.pose.position.y = (lat - lat0) * m_per_lat
            waypoints.poses.append(waypoint)
    rospy.loginfo(waypoints)
    waypoints.header.stamp = rospy.get_rostime()
    waypoints.header.frame_id = "/world"
    while not rospy.is_shutdown():
        pub.publish(waypoints)
        rate.sleep()

if __name__ == '__main__':
    try:
        load_points()
    except rospy.ROSInterruptException:
        pass