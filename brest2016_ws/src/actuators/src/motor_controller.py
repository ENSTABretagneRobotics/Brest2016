#!/usr/bin/env python
import maestro
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('motors')


def set_cmd(msg):
    speed = msg.linear.x
    rot = msg.angular.z
    print 'speed: ', speed, ' rot:', rot
    servo.setTarget(2, int(speed))
    servo.setTarget(1, int(rot))


def set_param_motor():
    """ Meilleur parametres pour les moteurs (accel, speed)"""
    servo.setSpeed(1, 0)    # max = 255
    servo.setAccel(1, 0)
    servo.setSpeed(2, 150)    # max = 255
    servo.setAccel(2, 150)


servo = maestro.Controller(0)    # faire attention au port
set_param_motor()

sub = rospy.Subscriber('cmd_vel', Twist, set_cmd)

rospy.spin()
