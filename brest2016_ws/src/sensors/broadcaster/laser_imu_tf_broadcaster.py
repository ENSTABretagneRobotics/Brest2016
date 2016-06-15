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
from sensors.msg import YPR


def update_transform(msg):
    " Updates the transform data that are going to be published"
    global quaternion
    quaternion = tf.transformations.quaternion_from_euler(msg.R, msg.P, msg.Y)

if __name__ == '__main__':
    rospy.init_node('laser2boat_tf_broadcaster')

    # Subscriber to the laser's imu
    sub = rospy.Subscriber('imu_laser', YPR, update_transform)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

    quaternion = [0, 0, 0, 1]
    # LOOP
    while not rospy.is_shutdown():
        br.sendTransform((100, 0.0, 0.0),
                         tuple(quaternion),
                         rospy.Time.now(),
                         "laser_frame",
                         "boat_frame")
        rate.sleep()
