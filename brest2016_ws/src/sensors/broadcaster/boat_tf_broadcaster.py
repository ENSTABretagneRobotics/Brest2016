#!/usr/bin/env python
# ########################################################
# This broadcaster will send the transform between the
# boat_frame --> world
#
# It will subscribe to the gps and imu on the boat
# and update the transform accordingly
# ########################################################
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
# from math import radians


def update_quaternion(msg):
    " Updates the transform data that are going to be published"
    global quat
    quat = msg.orientation
    rospy.loginfo(msg.orientation)
    # quat = tf.transformations.quat_from_euler(
    #     radians(msg.R), radians(msg.P), radians(msg.Y))


def update_pose(msg):
    " Updates the transform data that are going to be published"
    global x, y, z
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z


if __name__ == '__main__':
    rospy.init_node('boat2world_tf_broadcaster')

    # Subscriber to the boat's imu
    sub_imu = rospy.Subscriber('imu_boat', Imu, update_quaternion)
    sub_gps = rospy.Subscriber('gps/local_pose', PoseStamped, update_pose)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

    quat = Quaternion()
    quat.w = 1.0
    x, y, z = 0, 0, 0
    # LOOP
    while not rospy.is_shutdown():
        # br.sendTransform((x, y, z),
        br.sendTransform((0, 0, 0),
                         (quat.x, quat.y, quat.z, quat.w),
                         rospy.Time.now(),
                         "boat_frame",
                         "world")
        rate.sleep()
