#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64
from math import atan2


def calc_desired_heading(msg):
    global desired_heading
    desired_heading = atan2(
        waypoint.position.y - msg.pose.position.y,
        waypoint.position.x - msg.pose.position.x)
    pub.publish(desired_heading)


def set_waypoint(msg):
    global waypoint
    waypoint = msg

rospy.init_node('waypoint_controller')

# Abonnement au positions GPS
sub_pos = rospy.Subscriber('gps/local_pose', PoseStamped, calc_desired_heading)

# Abonnement a la position de la cible
sub_target = rospy.Subscriber('waypoint/local_pose', Pose, set_waypoint)

# Publisher de heading
pub = rospy.Publisher('robot/cap_desire', Float64, queue_size=1)

desired_heading = Float64()
waypoint = Pose()

rospy.spin()
