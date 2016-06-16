/* 
 * rosserial LIDAR-Lite v2 "Blue Label"
 * 
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
// Includes for Lidar
#include <Wire.h>
#include <LIDARLite.h>

ros::NodeHandle  nh;


sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

LIDARLite myLidarLite;

const int analog_pin = 0;
unsigned long range_timer;

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 */
float getRange(int pin_num){
    return float(myLidarLite.distance())/100.0;
}

char frameid[] = "/ir_ranger";

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;
  range_msg.max_range = 0.4;


//  Serial.begin(57600);
  myLidarLite.begin();
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    range_msg.range = getRange(analog_pin);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis() + 50;
  }
  nh.spinOnce();
}

