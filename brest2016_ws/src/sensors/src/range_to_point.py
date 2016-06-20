#!/usr/bin/env python

# @author Alaa El Jawad
# ########################################################
# This node transform the range read by the laser to a
# 3D point in the frame of the laser
# ########################################################

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped
import tf


def publish_point(msg):
    " Publishes the range as a point in the laser_frame"
    global listener
    ps = PointStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'laser_frame'
    ps.point.x = msg.range

    # log = 'Range: {} ... Point.x: {}'.format(msg.range, ps.point.x)
    # rospy.loginfo(log)

    pub.publish(ps)

    # wait for transform takes too long
    # listener.waitForTransform(
    #     "laser_frame", "boat_frame", rospy.Time().now(), rospy.Duration(1))
    # ps2 = listener.transformPoint('boat_frame', ps)
    # listener.waitForTransform(
    #     "laser_frame", "world", rospy.Time().now(), rospy.Duration(1))
    # ps3 = listener.transformPoint('world', ps)

    # pub2.publish(ps2)
    # pub3.publish(ps3)


if __name__ == '__main__':
    rospy.init_node('range_to_point')
    listener = tf.TransformListener()
    # listener.waitForTransform(
    #     "laser_frame", "boat_frame", rospy.Time(), rospy.Duration(5))

    # Subscriber to the laser's imu
    sub = rospy.Subscriber('range_data', Range, publish_point)
    # Publisher
    pub = rospy.Publisher('laser_point', PointStamped, queue_size=1)
    # pub2 = rospy.Publisher('laser_point_boat_frame',
    #                        PointStamped, queue_size=1)
    # pub3 = rospy.Publisher('laser_point_world_frame',
    #                        PointStamped, queue_size=1)

    rospy.spin()
