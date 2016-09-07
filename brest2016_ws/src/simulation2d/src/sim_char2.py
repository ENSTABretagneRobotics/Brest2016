#!/usr/bin/env python
import sys
sys.path.insert(
    0, '/home/ejalaa12/Desktop/Brest2016/brest2016_ws/src/simulation2d/models')
from char import Char
import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Twist
import tf
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm


def update_cmd(msg):
    """ Met a jour la commande en utilisant le vecteur cible """
    global u, char
    vect = np.array([msg.x, msg.y])
    u[1] = norm(vect)
    thetabar = np.arctan2(msg.y, msg.x)
    if u[1] == 0:
        u[0] = 0
    else:
        u[0] = 3 * np.arctan(np.tan(0.5 * (thetabar - char.theta)))
    # print 'thetabar', thetabar, 'u0', u[0]
    # print x[2], vect, thetabar
    print 'command:', thetabar, char.theta, u
    # print u


def update_cmd2(msg):
    """ Met a jour la commande avec celle recue """
    global u
    print 'Received:', msg.linear.x, msg.angular.z
    u[1] = abs(msg.linear.x / 6000.) - 1.0
    u[0] = msg.angular.z / 6000. - 1.0
    # augmenter la commande
    u[1] *= 10
    u[0] *= 5


def publish_pose():
    global char
    pos = PoseStamped()
    pos.header.frame_id = 'world'
    pos.header.stamp = rospy.Time.now()
    pos.pose.position.x = char.x
    pos.pose.position.y = char.y
    pos.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, char.theta)
    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

# Initialisation du noeud
rospy.init_node('sim_char')

# Subscriber et publisher
cmd_sub_v = rospy.Subscriber('robot/vecteur_cible', Vector3, update_cmd)
cmd_sub_u = rospy.Subscriber('cmd_vel_ramped', Twist, update_cmd2)
pos_pub = rospy.Publisher('gps/local_pose', PoseStamped, queue_size=1)

rate = rospy.Rate(20)
plt.ion()

char = Char(40, 40, 4)
# x = np.array([40, 40, 4])
dt = 0.1
u = [0, 0]

while not rospy.is_shutdown():
    # plt.cla()
    char.simulate(u)
    # x = x + fdot(x, u) * dt
    # draw_tank(x)
    # plt.axis([-30, 30, -30, 30])
    # plt.draw()
    publish_pose()
    print '-' * 10
    print 'u:', u
    print 'x', char.x, char.y, char.theta
    print
    rate.sleep()
