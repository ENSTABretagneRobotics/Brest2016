#!/usr/bin/env python
"""
ROS Node to log GPS data (NavSatFix format) to a CSV file
"""

import rospy
from sensor_msgs.msg import NavSatFix


def create_csv_file():
    f = open('gps.csv', 'w')
    f.write('type,latitude,longitude,alt\n')
    f.close()


def callback(msg):
    print type(msg.latitude), type(msg.longitude)
    with open('gps.csv', 'a') as f:
        f.write('T,')
        f.write(str(msg.latitude))
        f.write(',')
        f.write(str(msg.longitude))
        f.write(',')
        f.write(str(msg.altitude))
        f.write('\n')


rospy.init_node('gps2csv')
create_csv_file()

sub = rospy.Subscriber('gps', NavSatFix, callback)

rospy.spin()
