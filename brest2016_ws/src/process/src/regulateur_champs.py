#!/usr/bin/env python

import tf
import rospy
import numpy as np
from math import cos
from geometry_msgs.msg import PoseStamped, Twist, Vector3


def update_cap(msg):
    global cap
    # print msg.pose.orientation, type(msg.pose.orientation)
    cap = tf.transformations.euler_from_quaternion(
        [msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w])[2]


def update_cible(msg):
    global cap_cible, vitesse_cible
    # print msg, type(msg)
    if msg.x == 0:
        if msg.y >= 0:
            cap_cible = np.pi / 2
        else:
            cap_cible = -np.pi / 2
    else:
        cap_cible = np.arctan2(msg.y, msg.x)

    vitesse_cible = np.sqrt(msg.x**2 + msg.y**2)

rospy.init_node('regulateur')

# Subscribes to gps position to calculate the heading
# S'abonne aux positions GPS pour pour recupere le cap
sub_cap = rospy.Subscriber("gps/local_pose", PoseStamped, update_cap)

# Subscribes to the publisher to get the desired heading
# Recuper le cap desire
sub_consigne = rospy.Subscriber("robot/vecteur_cible", Vector3, update_cible)

# Publish the commands
# Publis les commandes
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

cap_cible = 0
cap = 0
vHigh = 6500  # 8000 max
vLow = 6500  # 6000 = vitesse nulle
K = 1500 * (2. / np.pi)
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    error = cap_cible - cap
    cmd = Twist()
    cmd.angular.z = 6000 + K * np.arctan(np.tan((error / 2.)))
    if cos(error) >= 0:
        print 'cos > 0'
        cmd.linear.x = vHigh
    else:
        print 'cos < 0'
        cmd.linear.x = vLow

    print cap, cap_cible, error, cos(error), "::", cmd.linear.x, cmd.angular.z
    cmd_pub.publish(cmd)
    rate.sleep()
