#!/usr/bin/env  python
import rospy
from process.srv import behavior, behaviorResponse
from geometry_msgs.msg import Vector3, PoseStamped
from Behavior import Behavior, BehaviorManager
import matplotlib.pyplot as plt
import numpy as np
import tf


def draw_tank(x):
    M = np.array([[1, -1, 0, 0, -1, -1, 0, 0, -1, 1, 0, 0, 3, 3, 0],
                  [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 0.5, -0.5, -1]])
    M = np.vstack((M, np.ones(M.shape[1])))
    R = np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]],
                  [np.sin(x[2]), np.cos(x[2]), x[1]],
                  [0, 0, 1]])

    M = np.dot(R, M)
    plt.plot(M[0], M[1])


def add_behavior(request):
    global listB
    print request.info.type, request.info.behavior_id
    behavior_manager.add_behavior(Behavior(request.info))
    print 'nombres de behavior recu:', len(behavior_manager.behavior_list)
    return behaviorResponse('oui')


def update_pos(msg):
    global x, y, cap
    x = msg.pose.position.x
    y = msg.pose.position.y
    quaternion = (msg.pose.orientation.x,
                  msg.pose.orientation.y,
                  msg.pose.orientation.z,
                  msg.pose.orientation.w)
    cap = tf.transformations.euler_from_quaternion(quaternion)[2]


# Variables
listB = []
x, y, cap = 0, 0, 0
behavior_manager = BehaviorManager()

# Initialisation du noeud
rospy.init_node('service_server')
# Initialisation du Serveur
service = rospy.Service('behavior_manager', behavior, add_behavior)
# Subscriber et publisher
pos_sub = rospy.Subscriber('gps/local_pose', PoseStamped, update_pos)
pub = rospy.Publisher('robot/vecteur_cible', Vector3, queue_size=1)

rate = rospy.Rate(10)

# Affichage du champ de vecteur
plt.ion()
cx, cy = 0, 0

while not rospy.is_shutdown():
    if abs(cx - x) > 30:
        cx = x
    if abs(cy - y) > 30:
        cy = y
    X, Y = np.mgrid[cx - 50:cx + 50:40j, cy - 50:cy + 50:40j]
    v = Vector3()
    v.x, v.y = behavior_manager.champ_total.cmd_point(x, y)
    U, V = behavior_manager.champ_total.cmd_point(X, Y)
    plt.cla()
    plt.quiver(X, Y, U, V)
    plt.quiver(x, y, v.x, v.y, color='red')
    draw_tank([x, y, cap])
    plt.draw()
    pub.publish(v)
    print v.x, v.y
    rate.sleep()
