#!/usr/bin/env python

import tf
import rospy
import numpy as np
from math import cos
from geometry_msgs.msg import PoseStamped, Twist, Vector3, TwistStamped
from std_msgs.msg import Float32MultiArray


def update_cap(msg):
    global cap, robot_type
    # print msg.pose.orientation, type(msg.pose.orientation)
    if robot_type == 'normal':
        cap = tf.transformations.euler_from_quaternion(
            [msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w])[2]
    elif robot_type == 'thomas_boat':
        cap = msg.data[0]


def update_speed(msg):
    global vitesse_boat
    vitesse_boat = msg.twist.linear.x


def update_cible(msg):
    global cap_cible, vitesse_cible
    # print msg, type(msg)
    cap_cible = np.arctan2(msg.y, msg.x)

    vitesse_cible = np.sqrt(msg.x**2 + msg.y**2)


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to' , default
        return default

rospy.init_node('regulateur')

robot_type = fetch_param('~robot_type', 'normal')
# Subscribes to gps position to calculate the heading
# S'abonne aux positions GPS pour pour recupere le cap
if robot_type == 'normal':
    sub_cap = rospy.Subscriber("gps/local_pose", PoseStamped, update_cap)
elif robot_type == 'thomas_boat':
    sub_cap = rospy.Subscriber("boat/compas", Float32MultiArray, update_cap)
    sub_speed = rospy.Subscriber(
        "boat/gps/velocity", TwistStamped, update_speed)

# Subscribes to the publisher to get the desired heading
# Recuper le cap desire
sub_consigne = rospy.Subscriber("robot/vecteur_cible", Vector3, update_cible)

# Publish the commands
# Publis les commandes
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

cap_cible = 0
vitesse_cible = fetch_param('~speed_zero', 6000)
vitesse_boat = 0
speed_zero = fetch_param('~speed_zero', 6000)
prout = fetch_param('~prout', 1)
cap = 0
vHigh = fetch_param('~vHigh', 8000)  # 8000 max
vLow = fetch_param('~vLow', 7000)  # 6000 = vitesse nulle
K = fetch_param('~K', 1500) * (2. / np.pi)
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    error = cap_cible - cap
    cmd = Twist()
    cmd.angular.z = speed_zero + K * np.arctan(np.tan((error / 2.)))
    if cos(error) >= 0:
        print 'cos > 0'
        cmd.linear.x = vHigh
    else:
        print 'cos < 0'
        if vitesse_cible > prout and vitesse_boat > 2:
            cmd.linear.x = -vHigh
        else:
            cmd.linear.x = vLow
    if vitesse_cible <= 0.1:
        cmd.linear.x = speed_zero
        cmd.angular.z = speed_zero

    print cap, cap_cible, error, cos(error), "::", cmd.linear.x, cmd.angular.z
    if robot_type == 'thomas_boat':
        cmd.angular.z = -cmd.angular.z
    cmd_pub.publish(cmd)
    rate.sleep()
