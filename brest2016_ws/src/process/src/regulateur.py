#!/usr/bin/env python

import tf
import rospy
import numpy as np
from math import cos
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Twist


def update_cap(msg):
    global cap
    # print msg.pose.orientation, type(msg.pose.orientation)
    cap = tf.transformations.euler_from_quaternion(
        [msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w])[2]


def callback2(msg):
    global cap_desire
    # print msg, type(msg)
    cap_desire = float(msg.data)

rospy.init_node('regulateur')

# Subscribes to gps position to calculate the heading
# S'abonne aux positions GPS pour pour recupere le cap
sub_cap = rospy.Subscriber("gps/local_pose", PoseStamped, update_cap)

# Subscribes to the publisher to get the desired heading
# Recuper le cap desire
sub_consigne = rospy.Subscriber("robot/cap_desire", Float64, callback2)

# Publish the commands
# Publis les commandes
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

cap_desire = 0
cap = 0
vHigh = 7500  # 8000 max
vLow = 6500  # 6000 = vitesse nulle
K = 1500 * (2. / np.pi)
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    error = cap_desire - cap
    cmd = Twist()
    cmd.angular.z = 6000 + K * np.arctan(np.tan((error / 2.)))
    if cos(error) >= 0:
        cmd.linear.x = vHigh
    else:
        cmd.linear.x = vLow

    print cap, cap_desire, error, "::", cmd.linear.x, cmd.angular.z
    cmd_pub.publish(cmd)
    rate.sleep()
