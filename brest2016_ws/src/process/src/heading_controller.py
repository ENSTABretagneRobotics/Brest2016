#!/usr/bin/env python

import rospy
from math import sin, cos
from std_msgs.msg import Float64
import tf
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

rospy.init_node('heading_controller')

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
K = 500
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    error = cap_desire - cap
    cmd = Twist()
    if cos(error) >= 0:
        cmd.linear.x = 7500
        cmd.angular.z = 6000 + K * sin(error)
    else:
        cmd.linear.x = 6000
        if sin(error) >= 0:
            cmd.angular.z = 4000
        else:
            cmd.angular.z = 8000
    print cap, cap_desire, error, "::", cmd.linear.x, cmd.angular.z
    cmd_pub.publish(cmd)
    rate.sleep()
