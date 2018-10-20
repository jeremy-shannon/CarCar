#!/usr/bin/env python
import rospy
import csv
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy

PATH_IS_RECORDING = False

class record_path_node:
    def __init__( self ):
        rospy.init_node( 'record_path_node', anonymous = True )
        rospy.Subscriber( "/tcpfix", NavSatFix, self.rtk_cb, csvWriter )
        rospy.Subscriber( "/joy", Joy, self.joy_cb )

        #I don't need "self." here right?
        csvFile = ""
        csvWriter = ""

        self.open_csv_file()
            
        #spinOnce maybe?
        rospy.spin()

    def open_csv_file( self ):
        #This should not be an absolute path
        self.csvFile = open( '/home/ubuntu/harcar_catkin_ws/src/load_path_node/path.csv', mode = 'w' )
        self.csvWriter = csv.writer( self.csvFile, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL )

    def rtk_cb( self, data ):
        if PATH_IS_RECORDING == True:
            self.csvWriter.writerow( [data.longitude, data.latitude] )
            rospy.loginfo( "Wrote to CSV: Lon = " + data.longitude + ", Lat = " + data.latitude )

    def joy_cb( self, data ):
        start_button = data.buttons[8]

        #This might fire twice. Once for button_down and again for button_up
        if start_button == True:
            if PATH_IS_RECORDING == False:
                PATH_IS_RECORDING = True
                self.open_csv_file()
            else:
                PATH_IS_RECORDING = False
                self.csvFile.close()

if __name__ == '__main__':
    #Should add the ability to shut down gracefully and close the csv file
    try:
        record_path_node()
    except rospy.ROSInterruptException:
        rospy.logerr( 'Could not start record path node.' )
