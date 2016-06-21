#!/usr/bin/env python

# envoie les commandes a la polulu


import maestro
import rospy

rospy.init_node('servo_laser')


def set_param_motor():
    """ Meilleur parametres pour les moteurs (accel, speed)"""
    servo.setSpeed(5, 30)    # max = 255
    servo.setAccel(5, 0)


servo = maestro.Controller(1)    # faire attention au port
set_param_motor()

cpt = 0
rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    if cpt % 2 == 0:
        servo.setTarget(5, -1)
    else:
        servo.setTarget(5, 1)
    cpt += 1
    print cpt
    rate.sleep()
