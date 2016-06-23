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
from std_msgs.msg import Header
import tf
import time
import rosbag


def convert_to_pcl2(msg):
    global pcl_points, listener, tot, bag

    if msg.range < 0.01:    # to filter out infinity
        return

    pts = PointStamped()
    pts.header.frame_id = 'laser_frame'
    pts.header.stamp = msg.header.stamp
    # print msg.header.stamp.secs,
    pts.point.x = msg.range
    # print pts.header.frame_id, pts.point.x, pts.point.y, pts.point.z,
    # print 'time:', pts.header.stamp.secs

    # start = time.time()
    listener.waitForTransform(
        "laser_frame", "world", rospy.Time().now(), rospy.Duration(0.5))
    pts2 = listener.transformPoint('world', pts)
    # diff = time.time() - start
    # tot += diff
    # print diff, tot
    # print pts2.header.frame_id, pts2.point.x, pts2.point.y, pts2.point.z

    pcl_points.append([pts2.point.x, pts2.point.y, pts2.point.z])
    # if len(pcl_points) > 200:
    #     pcl_points.pop(0)

    pcl = pc2.create_cloud_xyz32(pts2.header, pcl_points[-201:-1])
    pub.publish(pcl)


rospy.init_node('range_to_pcl')
# rosbag
filename = 'pcl-' + str(time.strftime("%Y-%m-%d-%H-%M-%S")) + '.bag'
print filename
bag = rosbag.Bag(filename, 'w')
listener = tf.TransformListener()

# Subscriber to the laser's imu
sub = rospy.Subscriber('range_data', Range, convert_to_pcl2)
pub = rospy.Publisher('laser_pcl', PointCloud2, queue_size=1)

pcl_points = []
tot = 0

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # s = String()
    # s.data = 'sent'
    # bag.write('log', s)
    if len(pcl_points) != 0:
        header = Header()
        header.frame_id = 'world'
        header.stamp = rospy.Time.now()
        pcl_tot = pc2.create_cloud_xyz32(header, pcl_points)
        bag.write('my_pcl', pcl_tot)
    rate.sleep()

bag.close()
