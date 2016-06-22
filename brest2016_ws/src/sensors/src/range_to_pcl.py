#!/usr/bin/env python

# @author Alaa El Jawad
# ########################################################
# This node transform the range read by the laser to a
# 3D point in a PointCloud2 msg
# ########################################################

import rospy
from sensor_msgs.msg import Range, PointCloud2
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2 as pc2
import tf
import time


def convert_to_pcl2(msg):
    global old_points, listener, tot

    # if msg.range < 0.01:    # to filter out infinity
    #     return

    pts = PointStamped()
    pts.header.frame_id = 'laser_frame'
    pts.header.stamp = msg.header.stamp
    print msg.header.stamp.secs,
    pts.point.x = msg.range
    # print pts.header.frame_id, pts.point.x, pts.point.y, pts.point.z,
    # print 'time:', pts.header.stamp.secs

    start = time.time()
    listener.waitForTransform(
        "laser_frame", "world", rospy.Time(0), rospy.Duration(0.05))
    pts2 = listener.transformPoint('world', pts)
    diff = time.time() - start
    tot += diff
    print diff, tot
    # print pts2.header.frame_id, pts2.point.x, pts2.point.y, pts2.point.z

    old_points.append([pts2.point.x, pts2.point.y, pts2.point.z])
    if len(old_points) > 10:
        old_points.pop(0)

    pcl = pc2.create_cloud_xyz32(pts2.header, old_points)
    pub.publish(pcl)


if __name__ == '__main__':
    rospy.init_node('range_to_pcl')
    listener = tf.TransformListener()

    # Subscriber to the laser's imu
    sub = rospy.Subscriber('range_data', Range, convert_to_pcl2)
    pub = rospy.Publisher('laser_pcl', PointCloud2, queue_size=1)

    old_points = []
    tot = 0

    rospy.spin()
