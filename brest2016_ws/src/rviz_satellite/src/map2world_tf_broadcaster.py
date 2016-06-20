#!/usr/bin/env python


# publie la transformee entre map et world


import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('map2world_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        br.sendTransform((100, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "map",
                         "world")
        rate.sleep()
