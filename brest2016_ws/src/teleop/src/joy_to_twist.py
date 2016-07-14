#!/usr/bin/env python
import rospy
# from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
# rosrun joy joy_node pour lancer
# Middle speed and rotations
speed = 6000
rot = 6000
max = 2000


def joy_cb(data, twist_pub):
    max = abs(data.axes[2] - 1) * 1000
    speed = data.axes[1] * max + 6000
    rot = data.axes[0] * max + 6000

    t = Twist()
    t.linear.x = speed
    t.angular.z = rot
    twist_pub.publish(t)

if __name__ == '__main__':
    rospy.init_node('joy_to_twist')
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('joy', String, joy_cb, twist_pub)
    rospy.spin()
