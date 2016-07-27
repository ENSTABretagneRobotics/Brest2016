#!/usr/bin/env python


# envoie le waypoint suivant


import rospy
from math import sqrt
from geometry_msgs.msg import Pose, PoseStamped


def genPose(x, y, z):
    pos = Pose()
    pos.position.x = x
    pos.position.y = y
    pos.position.z = z
    return pos


def calc_distance(msg):
    global current_waypoint, distance, current_index, waypoints
    distance = sqrt((current_waypoint.position.x - msg.pose.position.x)**2 + (current_waypoint.position.y - msg.pose.position.y)**2)
    if distance < capture_radius:
        current_index += 1
        current_waypoint = waypoints[current_index]
    wp_pub.publish(current_waypoint)
    print distance


rospy.init_node('waypoint_controller')

# Subscriber to GPS local position
gps_sub = rospy.Subscriber('gps/local_pose', PoseStamped, calc_distance)

# Publisher for waypoint
wp_pub = rospy.Publisher('waypoint/local_pose', Pose, queue_size=1)

waypoints = [genPose(40, 40, 0), genPose(34, 118, 0), genPose(61, 85, 0)]
distance = 10000
current_index = 0
capture_radius = 5
current_waypoint = waypoints[0]

rospy.spin()
