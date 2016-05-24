#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {
    'z': [1, 0],  # avant
    'q': [0, 1],  # gauche
    's': [0, 0],    # stop
    'd': [0, -1],  # droite
    'x': [-1, 0]  # arriere
}

# Middle speed and rotations
speed = 0
rot = 0


def keys_cb(msg, twist_pub):
    global speed, rot
    if len(msg.data) == 0 or msg.data not in key_mapping.keys():
        return  # nothing to do, unknown key
    vels = key_mapping[msg.data[0]]
    if msg.data == 's':
        # we reset the speed and rot
        speed = 0
        rot = 0
    else:
        speed += vels[0]
        rot += vels[1]

    t = Twist()
    t.linear.x = speed
    t.angular.z = rot
    twist_pub.publish(t)

if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_cb, twist_pub)
    rospy.spin()
