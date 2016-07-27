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
from geometry_msgs.msg import PoseStamped, Vector3
from numpy import arctan2
from math import degrees


def update_cible(msg):
    """
    Met a jour la vitesse reelle et le cap cible a partir
    du topic /robot/vecteur_cible
    """
    global pos

    cap_cible = arctan2(msg.y, msg.x)    # cap desire
    print degrees(cap_cible)
    quat = tf.transformations.quaternion_from_euler(0, 0, cap_cible)
    pos.pose.orientation.x = quat[0]
    pos.pose.orientation.y = quat[1]
    pos.pose.orientation.z = quat[2]
    pos.pose.orientation.w = quat[3]
    # print type(pos.pose.orientation)


rospy.init_node('target_vector_pose')
rate = rospy.Rate(10.0)

# Subscriber to the boat's imu
sub_consigne = rospy.Subscriber(
    "robot/vecteur_cible", Vector3, update_cible)

# Publisher of target vector pose
pub = rospy.Publisher('robot/vecteur_cible_pose',
                      PoseStamped, queue_size=1)

pos = PoseStamped()
pos.header.frame_id = 'boat_frame'

# LOOP
while not rospy.is_shutdown():
    pos.header.stamp = rospy.Time.now()
    print 'lksdj'
    pub.publish(pos)
    rate.sleep()
