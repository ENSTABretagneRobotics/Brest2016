#!/usr/bin/env python

# Lat Long - UTM, UTM - Lat Long conversions

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from math import pi, cos

LAT0 = 48.417753
LON0 = -4.474413
R = 6371000


def deg2rad(deg):
    return deg * pi / 180


def ll2local(lat0, lon0, lat, lon, rho):
    x = rho * deg2rad(lat - lat0)
    y = rho * cos(deg2rad(lat)) * deg2rad(lon - lon0)
    return [x, y]


def callback(msg):
    y, x = ll2local(LAT0, LON0, msg.latitude, msg.longitude, R)
    print x, y

    pos = PoseStamped()
    pos.header.frame_id = 'world'
    pos.header.stamp = rospy.Time.now()
    pos.pose.position.x = x
    pos.pose.position.y = y
    pos.pose.position.z = 0
    pos.pose.orientation.x = 0
    pos.pose.orientation.y = 0
    pos.pose.orientation.z = 0
    pos.pose.orientation.w = 1
    pub.publish(pos)


rospy.init_node('Local_publisher')

sub = rospy.Subscriber('gps', NavSatFix, callback)
pub = rospy.Publisher('gps/local_pose', PoseStamped, queue_size=1)

rospy.spin()
