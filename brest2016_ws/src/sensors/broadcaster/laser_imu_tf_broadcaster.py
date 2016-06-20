#!/usr/bin/env python
# ########################################################
# This broadcaster will send the transform between the
# laser_frame --> the boat_frame
#
# It will subscribe to the imu that is on the laser
# and update the transform accordingly
# ########################################################
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
# from math import radians


def update_transform(msg):
    " Updates the transform data that are going to be published"
    global quat
    quat = msg.orientation
    # quat = tf.transformations.quat_from_euler(
    #     radians(msg.R), radians(msg.P), radians(msg.Y))


def update_pose(msg):
    " Updates the transform data that are going to be published"
    global x, y, z
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

if __name__ == '__main__':
    rospy.init_node('laser2boat_tf_broadcaster')

    # Subscriber to the laser's imu
    sub = rospy.Subscriber('imu_laser', Imu, update_transform)
    sub_gps = rospy.Subscriber('gps/local_pose', PoseStamped, update_pose)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

    quat = Quaternion()
    quat.w = 1.0

    x, y, z = 0, 0, 0
    # LOOP
    while not rospy.is_shutdown():
        br.sendTransform((x, y, z),
        # br.sendTransform((0.0, 0.0, 0.0),
                         (quat.x, quat.y, quat.z, quat.w),
                         rospy.Time.now(),
                         "laser_frame",
                         "world")
        rate.sleep()
