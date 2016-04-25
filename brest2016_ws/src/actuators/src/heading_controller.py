#!/usr/bin/env python

import rospy
from math import sin, pi
from std_msgs.msg import Float64
import tf
from geometry_msgs.msg import PoseStamped, Twist


def update_cap(msg):
    global cap
    print msg.pose.orientation, type(msg.pose.orientation)
    cap = tf.transformations.euler_from_quaternion(
        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]


def callback2(msg):
    global cap_desire
    cap_desire = cap_desire

rospy.init_node('heading_controller')
sub_cap = rospy.Subscriber("gps/local_pose", PoseStamped, update_cap)
sub_consigne = rospy.Subscriber("robot/cap_desire", Float64, callback2)
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

cap_desire = 0
cap = 0
vitesse_desire = 7000
K = 500
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    error = cap_desire - cap
    cmd = Twist()
    cmd.linear.x = vitesse_desire
    print 6000 + error * K
    cmd.angular.z = 6000 + error * K
    cmd_pub.publish(cmd)
    rate.sleep()
