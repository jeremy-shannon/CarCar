#!/usr/bin/env python
import csv
import rospy
from harcar_msgs.msg import Path
from geometry_msgs.msg import Pose

def load_points():
	pub = rospy.Publisher('waypoint_path', Path, queue_size=10)
	rospy.init_node('load_path', anonymous=True)
	rate = rospy.Rate(0.2) #Hz. Once every 5 seconds.
	waypoints = []
	with open('/home/ubuntu/harcar_catkin_ws/src/load_path_node/Figure8_spaced30.csv') as csvfile:
		readCSV = csv.reader(csvfile, delimiter=',')
		for row in readCSV:
			waypoint = Pose()
			waypoint.position.x = float(row[1])
			waypoint.position.y = float(row[0])
			waypoints.append(waypoint)
	rospy.loginfo(waypoints)
	while not rospy.is_shutdown():
		pub.publish(waypoints)
		rate.sleep()

if __name__ == '__main__':
	try:
		load_points()
	except rospy.ROSInterruptException:
		pass