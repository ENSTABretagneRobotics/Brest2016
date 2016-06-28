#!/usr/bin/env python

# envoie les commandes a la polulu


import maestro
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('motors')


def set_cmd(msg):
    global max_speed, min_speed, max_rot, min_rot
    # Limiting speed
    if msg.linear.x <= min_speed:
        msg.linear.x = min_speed
    elif msg.linear.x >= max_speed:
        msg.linear.x = max_speed

    # Limiting rotation
    if msg.angular.z <= min_rot:
        msg.angular.z = min_rot
    elif msg.angular.z >= max_rot:
        msg.angular.z = max_rot

    speed = msg.linear.x
    rot = msg.angular.z

    print 'speed: ', speed, ' rot:', rot
    servo.setTarget(1, int(speed))
    servo.setTarget(2, int(rot))


def set_param_motor():
    """ Meilleur parametres pour les moteurs (accel, speed)"""
    servo.setSpeed(1, 0)    # max = 255
    servo.setAccel(1, 0)
    servo.setSpeed(2, 150)    # max = 255
    servo.setAccel(2, 150)


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to %.3f' % default
        return default

# Getting params
max_speed = fetch_param("~max_speed", 8000)
min_speed = fetch_param("~min_speed", 4000)
max_rot = fetch_param("~max_rot", 8000)
min_rot = fetch_param("~min_rot", 4000)

servo = maestro.Controller(0)    # faire attention au port
set_param_motor()

sub = rospy.Subscriber('cmd_vel_ramped', Twist, set_cmd)

rospy.spin()
