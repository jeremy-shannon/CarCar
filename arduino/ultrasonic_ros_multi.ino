#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#define USE_USBCON
//rosrun rosserial_python serial_node.py /dev/ttyACM0

ros::NodeHandle  nh;

sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
ros::Publisher pub_range_1( "/ultrasonic_1", &range_msg_1);
ros::Publisher pub_range_2( "/ultrasonic_2", &range_msg_2);
ros::Publisher pub_range_3( "/ultrasonic_3", &range_msg_3);

int triggerPin[] = { 2, 3, 4 };
int echoPin[] = { 8, 9, 10 };
long range_time = 0;
char frameid_1[] = "/ultrasonic_1";
char frameid_2[] = "/ultrasonic_2";
char frameid_3[] = "/ultrasonic_3";


void setup() {
  nh.initNode();
  nh.advertise(pub_range_1);
  nh.advertise(pub_range_2);
  nh.advertise(pub_range_3);
  
  range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_1.header.frame_id =  frameid_1;
  range_msg_1.field_of_view = 0.1;  // fake
  range_msg_1.min_range = 3.0;
  range_msg_1.max_range = 600.47;

  range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_2.header.frame_id =  frameid_2;
  range_msg_2.field_of_view = 0.1;  // fake
  range_msg_2.min_range = 3.0;
  range_msg_2.max_range = 600.47;

  range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_3.header.frame_id =  frameid_3;
  range_msg_3.field_of_view = 0.1;  // fake
  range_msg_3.min_range = 3.0;
  range_msg_3.max_range = 600.47;
}

void loop()
{
  if ( millis() >= range_time ){
      range_msg_1.range = getRange(0) * 1;
      range_msg_1.header.stamp = nh.now();
      pub_range_1.publish(&range_msg_1);

      range_msg_2.range = getRange(1) * 1;
      range_msg_2.header.stamp = nh.now();
      pub_range_2.publish(&range_msg_2);

      range_msg_3.range = getRange(2) * 1;
      range_msg_3.header.stamp = nh.now();
      pub_range_3.publish(&range_msg_3);
      range_time =  millis() + 50;
    }
    nh.spinOnce();
}

long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29.1 / 2;
}

float getRange( int sensorID )
{
    
    // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(triggerPin[sensorID], OUTPUT);
  digitalWrite(triggerPin[sensorID], LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin[sensorID], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin[sensorID], LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin[sensorID], INPUT);
  duration = pulseIn(echoPin[sensorID], HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}
