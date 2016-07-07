#!/usr/bin/env python


# simule un char


import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Twist
import tf
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm


def draw_tank(x):
    M = np.array([[1, -1, 0, 0, -1, -1, 0, 0, -1, 1, 0, 0, 3, 3, 0],
                  [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 0.5, -0.5, -1]])
    M = np.vstack((M, np.ones(M.shape[1])))
    R = np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]],
                  [np.sin(x[2]), np.cos(x[2]), x[1]],
                  [0, 0, 1]])

    M = np.dot(R, M)
    plt.plot(M[0], M[1])


def fdot(x, u):
    xdot = u[1] * np.cos(x[2])
    ydot = u[1] * np.sin(x[2])
    thetadot = u[0]
    return np.array([xdot, ydot, thetadot])


def update_cmd(msg):
    """ Met a jour la commande en utilisant le vecteur cible """
    global u, x
    vect = np.array([msg.x, msg.y])
    u[1] = norm(vect)
    thetabar = np.arctan2(msg.y, msg.x)
    if u[1] == 0:
        u[0] = 0
    else:
        u[0] = 3 * np.arctan(np.tan(0.5 * (thetabar - x[2])))
    # print 'thetabar', thetabar, 'u0', u[0]
    # print x[2], vect, thetabar
    print 'command:', thetabar, x[2], u
    # print u


def update_cmd2(msg):
    """ Met a jour la commande en utilisant le vecteur cible """
    global u
    u[1] = msg.linear.x
    thetabar = msg.angular.z
    # print 'thetabar', thetabar, 'u0', u[0]
    # print x[2], vect, thetabar
    print 'command:', thetabar, x[2], u
    # print u


def publish_pose(x):
    pos = PoseStamped()
    pos.header.frame_id = 'world'
    pos.header.stamp = rospy.Time.now()
    pos.pose.position.x = x[0]
    pos.pose.position.y = x[1]
    pos.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, x[2])
    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

# Initialisation du noeud
rospy.init_node('sim_char')
# Subscriber et publisher
# cmd_sub = rospy.Subscriber('robot/vecteur_cible', Vector3, update_cmd)
cmd_sub = rospy.Subscriber('cmd_vel', Twist, update_cmd)
pos_pub = rospy.Publisher('gps/local_pose', PoseStamped, queue_size=1)

rate = rospy.Rate(10)
plt.ion()
x = np.array([40, 40, 4])
dt = 0.1
u = [0, 0]

while not rospy.is_shutdown():
    # plt.cla()
    x = x + fdot(x, u) * dt
    # draw_tank(x)
    # plt.axis([-30, 30, -30, 30])
    # plt.draw()
    publish_pose(x)
    print '-' * 10
    print 'u:', u
    print 'x', x
    print
    rate.sleep()

# if __name__ == '__main__':
#     for t in np.arange(1, 30, dt):
#         plt.cla()
#         x = x + fdot(x, u) * dt
#         draw_tank(x)
#         plt.axis([-30, 30, -30, 30])
#         plt.draw()
