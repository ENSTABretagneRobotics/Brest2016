#!/usr/bin/env python


# envoie le waypoint suivant


import rospy
from math import sqrt
from geometry_msgs.msg import PointStamped
from process.srv import behavior
from process.msg import BehaviorInfo


def sendWaypoint(x, y):
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='001', f_type='waypoint',
                        xa=x, ya=y,
                        xb=0, yb=0,
                        K=1, R=1,
                        security='LOW', slowing_R=1,
                        slowing_K=1,
                        effect_range=1)
    confirmation = behavior_sender(info, 'update')
    # print confirmation


def calc_distance(msg):
    global current_waypoint, distance, current_index, waypoints
    distance = sqrt((current_waypoint[0] - msg.point.x)**2 +
                    (current_waypoint[1] - msg.point.y)**2)
    if distance < capture_radius:
        current_index += 1
        current_index %= len(waypoints)
        current_waypoint = waypoints[current_index]
        print len(waypoints)
    sendWaypoint(current_waypoint[0], current_waypoint[1])
    print distance, current_index


rospy.init_node('behavior_waypoint_controller')

# Subscriber to GPS local position
gps_sub = rospy.Subscriber('boat/gps/pose', PointStamped, calc_distance)

# Service for sending waypoints
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

waypoints = [(-44, -195, 0), (-60, -60, 0)]
distance = 10000
current_index = 0
capture_radius = 5
current_waypoint = waypoints[0]

rospy.spin()
