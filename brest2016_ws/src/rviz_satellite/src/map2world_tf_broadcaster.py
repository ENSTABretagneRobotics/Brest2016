#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('map2world_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2000000.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "map",
                         "world")
        rate.sleep()
