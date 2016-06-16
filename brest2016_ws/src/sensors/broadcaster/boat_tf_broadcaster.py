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
from sensors.msg import YPR
from geometry_msgs.msg import PoseStamped
from math import radians


def update_quaternion(msg):
    " Updates the transform data that are going to be published"
    global quaternion
    quaternion = tf.transformations.quaternion_from_euler(
        radians(msg.R), radians(msg.P), radians(msg.Y))


def update_pose(msg):
    " Updates the transform data that are going to be published"
    global x, y, z
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z


if __name__ == '__main__':
    rospy.init_node('boat2world_tf_broadcaster')

    # Subscriber to the boat's imu
    sub_imu = rospy.Subscriber('imu_boat', YPR, update_quaternion)
    sub_gps = rospy.Subscriber('gps/local_pose', PoseStamped, update_pose)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

    quaternion = [0, 0, 0, 1]
    x, y, z = 0, 0, 0
    # LOOP
    while not rospy.is_shutdown():
        # br.sendTransform((x, y, z),
        br.sendTransform((0, 0, 0),
                         tuple(quaternion),
                         rospy.Time.now(),
                         "boat_frame",
                         "world")
        rate.sleep()
