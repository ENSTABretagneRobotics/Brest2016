#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {
    'w': [0, 1],  # avant
    'a': [1, 0],  # gauche
    's': [0, 0],    # stop
    'd': [-1, 0],  # droite
    'x': [0, -1]  # arriere
}

g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_twist_send_time = None
g_vel_scales = [0.1, 0.1]
g_vel_ramps = [1, 1]


def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    # compute maximum velocity step
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:
        return v_target
    else:
        return v_prev + sign * step


def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(
        prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
    tw.linear.x = ramped_vel(
        prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
    return tw


def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist
    global g_vel_scales, g_vel_ramps, g_twist_pub
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(
        g_last_twist, g_target_twist, g_last_twist_send_time,
        t_now, g_vel_ramps)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)
    return 0


def cmd_received(msg):
    global g_last_twist, g_target_twist, g_vel_scales
    print msg
    g_target_twist = msg
    # g_last_twist.linear.x = vels[1] * g_vel_scales[1]
    # g_last_twist.angular.z = vels[0] * g_vel_scales[0]


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to %.3f' % default
        return default


if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    g_last_twist_send_time = rospy.Time.now()
    g_twist_pub = rospy.Publisher('cmd_vel/ramped', Twist, queue_size=1)
    rospy.Subscriber('cmd_vel', Twist, cmd_received)
    g_target_twist = Twist()
    g_last_twist = Twist()
    g_vel_scales[0] = fetch_param('~angular_scale', 0.1)
    g_vel_scales[1] = fetch_param('~linear_scale', 0.1)
    g_vel_ramps[0] = fetch_param('~angular_accel', 1.0)
    g_vel_ramps[1] = fetch_param('~linear_accel', 1.0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()
