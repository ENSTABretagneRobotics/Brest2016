#!/usr/bin/env python

# @author Alaa El Jawad
# ########################################################
# This node transform the range read by the laser to a
# 3D point in the frame of the laser
# ########################################################

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped


def publish_point(msg):
    " Publishes the range as a point in the laser_frame"
    point = PoseStamped()
    point.header.time = rospy.Time.now()
    point.header.frame_id = 'laser_frame'
    point.pose.x = msg.range

    pub.publish(point)


if __name__ == '__main__':
    rospy.init_node('range_to_point')

    # Subscriber to the laser's imu
    sub = rospy.Subscriber('range_data', Range, publish_point)
    # Publisher
    pub = rospy.Publisher('laser_point', PoseStamped)
