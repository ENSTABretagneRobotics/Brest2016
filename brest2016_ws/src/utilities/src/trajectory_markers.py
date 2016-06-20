#!/usr/bin/env python


# cree des markers sur la carte pour recuperer la trajectoire du robot


"""
ROS Node to plot the trajectory of the robot on RViz using Markers
"""

import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker


def add_new_marker(msg):
    global marker, count
    point = Point()
    point.x = msg.pose.position.x
    point.y = msg.pose.position.y
    point.z = msg.pose.position.z
    marker.points.append(point)

    # marker_array.markers.append(marker)
    marker_pub.publish(marker)
    # count += 1

# Init Node
rospy.init_node('robot_path_plotter')

# Publisher and subscriber
pos_sub = rospy.Subscriber('gps/local_pose', PoseStamped, add_new_marker)
marker_pub = rospy.Publisher('robot_path', Marker, queue_size=1)

# Marker Array and count (global variables)
marker = Marker()
marker.header.frame_id = 'world'
# marker.header.stamp = rospy.Time.now()
marker.id = 0
marker.type = marker.LINE_STRIP
marker.action = marker.ADD
marker.scale.x = 0.5
# marker.scale.y = 0.2
# marker.scale.z = 0.2
marker.color.a = 1.0
marker.pose.orientation.w = 1.0
# marker.pose.position.x = msg.pose.position.x
# marker.pose.position.y = msg.pose.position.y
# marker.pose.position.z = msg.pose.position.z
count = 0

rospy.spin()

# # Rate
# rate = rospy.Rate(1)


# # LOOP
# while not rospy.is_shutdown():
#     marker_pub.publish(marker_array)
#     print len(marker_array.markers)
#     rate.sleep()
