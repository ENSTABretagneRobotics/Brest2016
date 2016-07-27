#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from math import degrees


def update_cap(msg):
    """
    Met a jour le cap du bateau en recuperant son cap a partir
    du topic /gps/local_pose
    """
    global cap

    # print msg.pose.orientation, type(msg.pose.orientation)
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    cap = tf.transformations.euler_from_quaternion(quat)[2]
    print degrees(cap)


# Ros initialisation and rate
rospy.init_node('regulateur')
rate = rospy.Rate(5)

# Subscribes to gps position to calculate the heading
# S'abonne aux positions GPS pour pour recupere le cap
sub_cap = rospy.Subscriber("imu_boat", Imu, update_cap)

rospy.spin()
