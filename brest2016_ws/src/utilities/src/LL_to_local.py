#!/usr/bin/env python


# conversion lat/long vers local
# renvoie l orientation du robot
# Lat Long - UTM, UTM - Lat Long conversions

import rospy
# import tf
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from math import pi, cos

# Point Reperable

# LAT0 = 48.417753    # ENSTA - bas gauche du terrain de foot
# LON0 = -4.474413
# LAT0 = 48.486964    # BOURG-BLANC - banc entree
# LON0 = -4.505771
# LAT0 = 48.492305    # BOURG-BLANC - quai
# LON0 = -4.503808
LAT0 = 48.3829633    # QUAI DES SCIENCES
LON0 = -4.48648166
R = 6371000


def deg2rad(deg):
    return deg * pi / 180


def ll2local(lat0, lon0, lat, lon, rho):
    x = rho * deg2rad(lat - lat0)
    y = rho * cos(deg2rad(lat)) * deg2rad(lon - lon0)
    return [x, y]


def update_pose(msg):
    global pos
    y, x = ll2local(LAT0, LON0, msg.latitude, msg.longitude, R)
    print x, y

    pos.header.frame_id = 'boat_frame'
    pos.header.stamp = rospy.Time.now()
    pos.pose.position.x = x
    pos.pose.position.y = y
    pos.pose.position.z = 0


def update_orientation(msg):
    global pos
    pos.pose.orientation = msg.orientation


rospy.init_node('Local_publisher')

sub_gps = rospy.Subscriber('gps', NavSatFix, update_pose)
sub_imu = rospy.Subscriber('imu', Imu, update_orientation)
pub = rospy.Publisher('gps/local_pose', PoseStamped, queue_size=1)

pos = PoseStamped()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(pos)
    rate.sleep()
