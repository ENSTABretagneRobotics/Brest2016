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
    cap_cible = np.arctan2(msg.y, msg.x)

    vitesse_cible = 6000+ 2000*(np.sqrt(msg.x**2 + msg.y**2)/5)


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to %.3f' % default
        return default

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
vitesse_cible = fetch_param('~speed_zero', 6000)
speed_zero = fetch_param('~speed_zero', 6000)
cap = 0
vHigh = fetch_param('~vHigh', 8000)  # 8000 max
vLow = fetch_param('~vLow', 7000)  # 6000 = vitesse nulle
K = fetch_param('~K', 1500) * (2. / np.pi)
alpha = fetch_param('~alpha', 0)
beta = fetch_param('~beta', -0.5)
rate = rospy.Rate(5)

# while not rospy.is_shutdown():
#     error = cap_cible - cap
#     cmd = Twist()
#     cmd.angular.z = speed_zero + K * np.arctan(np.tan((error / 2.)))
#     if cos(error) >= 0:
#         print 'cos > 0'
#         cmd.linear.x = vHigh
#     else:
#         print 'cos < 0'
#         cmd.linear.x = vLow
#     if vitesse_cible <= 0.1:
#         cmd.linear.x = speed_zero
#         cmd.angular.z = speed_zero



while not rospy.is_shutdown():
    error = cap_cible - cap
    cmd = Twist()

    # commande de la vitesse angulaire.
    if np.abs(error) <= beta :
        cmd.angular.z = speed_zero + K * np.arctan(np.tan((error / 2.)))
    else:
        cmd.angular.z = -(speed_zero + K * np.arctan(np.tan((error / 2.))))

    # commande de la vitesse.

    if vitesse_cible <=0.1
        V = 0
    elif vitesse_cible <= vLow:
        V = vLow
    elif vitesse_cible >= vHigh:
        V = vHigh
    else:
        V = vitesse_cible


    if cos(error) >= alpha:
        print 'cos > alpha'
        cmd.linear.x = V * cos(error)
    elif cos(error) <= beta:
        print 'cos < beta'
        cmd.linear.x = -V
    else : 
        cmd.linear.x = vLow


    print cap, cap_cible, error, cos(error), "::", cmd.linear.x, cmd.angular.z
    cmd_pub.publish(cmd)
    rate.sleep()
