#!/usr/bin/env python
import sys
sys.path.insert(
    0, '/home/ejalaa12/Desktop/Brest2016/brest2016_ws/src/simulation2d/models')
from sailboat import Sailboat
import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Twist
import tf
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm


def update_cmd(msg):
    """ On recoit le vecteur cible et on deduit la commande """
    global u, sailboat, awind, psi
    vect = np.array([msg.x, msg.y])
    u = sailboat.control(vect, awind, psi)


def manual_cmd(msg):
    global u
    u = np.array([msg.x, msg.y])


def publish_pose():
    global char
    pos = PoseStamped()
    pos.header.frame_id = 'world'
    pos.header.stamp = rospy.Time.now()
    pos.pose.position.x = sailboat.x
    pos.pose.position.y = sailboat.y
    pos.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, sailboat.theta)
    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

# Initialisation du noeud
rospy.init_node('sim_sailboat')

# Subscriber et publisher
cmd_sub = rospy.Subscriber('robot/vecteur_cible', Vector3, update_cmd)
cmd_manual = rospy.Subscriber('sailboat/manual', Vector3, manual_cmd)
pos_pub = rospy.Publisher('gps/local_pose', PoseStamped, queue_size=1)

rate = rospy.Rate(20)
plt.ion()

sailboat = Sailboat(0., 0., 0.1, 0., 0.)
awind = rospy.get_param('awind', 2)
psi = rospy.get_param('psi', np.pi / 2)
# x = np.array([40, 40, 4])
dt = 0.1
u = [0, 0]

while not rospy.is_shutdown():
    plt.cla()
    sailboat.simulate(u, awind, psi)
    sailboat.draw()
    sailboat.drawWind(awind, psi, coeff=5)
    plt.axis([sailboat.x - 10, sailboat.x + 10,
              sailboat.y - 10, sailboat.y + 10])
    # plt.axis([-100, 100, -100, 100])
    plt.draw()
    publish_pose()
    print '-' * 10
    print 'u:', u
    print 'x', sailboat.x, sailboat.y, sailboat.theta
    print
    rate.sleep()
