#!/usr/bin/env python
import rospy
import csv
from sensor_msgs.msg import NavSatFix

class record_path_node:
    def __init__(self):
        rospy.init_node('record_path_node', anonymous=True)

		#This should not be an absolute path
		csvfile = open('/home/ubuntu/harcar_catkin_ws/src/load_path_node/path.csv', mode='w')
		csvWriter = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

		rospy.Subscriber("/tcpfix", NavSatFix, self.rtk_cb, csvWriter)
			
		rospy.spin()

    def rtk_cb(self, data, csvWriter):
		csvWriter.writerow([data.longitude,data.latitude])

if __name__ == '__main__':
    try:
        record_path_node()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start record path node.')
