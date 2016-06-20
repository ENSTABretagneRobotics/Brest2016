#!/usr/bin/env  python

# recoit les behaviors,
# s'abonne a la position GPS
# afin de publier le vecteur a suivre


import rospy
from process.srv import behavior, behaviorResponse
from geometry_msgs.msg import Vector3, PoseStamped, PointStamped
from std_msgs.msg import Float32MultiArray
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


def handle_received_behavior(request):
    global listB
    print request.info.f_type, request.info.behavior_id
    behavior_manager.handle_behavior(Behavior(request.info), request.action)
    return behaviorResponse(request.action + 'ed')

    print 'nombres de behavior recu:', len(behavior_manager.behavior_list)


def update_pos(msg):
    global x, y, cap, robot_type
    if robot_type == 'normal':
        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = (msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w)
        cap = tf.transformations.euler_from_quaternion(quaternion)[2]
    elif robot_type == 'thomas_boat':
        x = msg.point.x
        y = msg.point.y


def update_cap(msg):
    global cap
    cap = msg.data[0]


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to', default
        return default

# Variables
listB = []
x, y, cap = 0, 0, 0
behavior_manager = BehaviorManager()

# Initialisation du noeud
rospy.init_node('service_server')
# Initialisation du Serveur
service = rospy.Service('behavior_manager', behavior, handle_received_behavior)
# Recuperation du type de robot
robot_type = fetch_param('~robot_type', 'normal')
# Subscriber et publisher
if robot_type == 'normal':
    pos_sub = rospy.Subscriber('gps/local_pose', PoseStamped, update_pos)
elif robot_type == 'thomas_boat':
    pos_sub = rospy.Subscriber('boat/gps/pose', PointStamped, update_pos)
    sub_cap = rospy.Subscriber("boat/compas", Float32MultiArray, update_cap)
pub = rospy.Publisher('robot/vecteur_cible', Vector3, queue_size=1)

rate = rospy.Rate(10)

# Affichage du champ de vecteur
plt.ion()
cx, cy = 0, 0

while not rospy.is_shutdown():
    if abs(cx - x) > 10:
        cx = x
    if abs(cy - y) > 10:
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
