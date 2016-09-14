#!/usr/bin/env  python

# recoit les behaviors,
# s'abonne a la position GPS
# afin de publier le vecteur a suivre


import rospy
from process.srv import behavior, behaviorResponse
from geometry_msgs.msg import Vector3, PoseStamped, PointStamped
from std_msgs.msg import Float32MultiArray
from utils.behavior import Behavior
from utils.behaviorManager import BehaviorManager
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

import sys
sys.path.insert(
    0, '/home/ejalaa12/Desktop/Brest2016/brest2016_ws/src/simulation2d/models')
from sailboat import Sailboat


# ##############################################################################

# ##############################################################################
def handle_received_behavior(request):
    # Handle Behavior
    wwd = behavior_manager.handle_behavior(
        Behavior(request.info), request.action)

    tmp = BehaviorManager(sailboat=True)
    tmp.handle_behavior(Behavior(request.info), request.action)
    print '>>' * 30,
    print tmp.champ_total.get_field(0, 10, 45, 100)
    # Log info
    log = 'RECEIVED::: MODE: {:<15}, ID: {:<4}, TYPE: {:<15}, TOTAL: {:<4}, X: {}, Y, {}'
    log = log.format(request.action, request.info.behavior_id,
                     request.info.f_type, len(behavior_manager.behavior_list),
                     request.info.xa, request.info.ya)
    rospy.loginfo(log)
    return behaviorResponse(wwd)


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
        print 'Defaulting to %.3f' % default
        return default
# ##############################################################################

# Variables
x, y, cap = 0, 0, 0
behavior_manager = BehaviorManager(sailboat=True)

# Environment parameters
awind = rospy.get_param('awind', 2)
psi = rospy.get_param('psi', np.pi / 2)
theta = rospy.get_param('no_go_zone_angle', 100)
wind = np.degrees(psi)

# Initialisation du noeud
rospy.init_node('behavior_server')

# Initialisation du Serveur
service = rospy.Service('behavior_manager', behavior, handle_received_behavior)

# Recuperation du type de robot
robot_type = rospy.get_param('~robot_type', 'normal')

# Subscriber et publisher
if robot_type == 'normal':
    pos_sub = rospy.Subscriber('gps/local_pose', PoseStamped, update_pos)
elif robot_type == 'thomas_boat':
    pos_sub = rospy.Subscriber('boat/gps/pose', PointStamped, update_pos)
    sub_cap = rospy.Subscriber("boat/compas", Float32MultiArray, update_cap)
pub = rospy.Publisher('robot/vecteur_cible', Vector3, queue_size=1)

rate = rospy.Rate(10)

# Affichage du champ de vecteur
show_plot = fetch_param('~show_plot', False)
print show_plot
if show_plot:
    plt.ion()
    trajx = []
    trajy = []
    ctr = 0
cx, cy = 0, 0
sb = Sailboat()

while not rospy.is_shutdown():
    # Publish command
    v = Vector3()
    v.x, v.y = behavior_manager.champ_total.get_field(x, y, wind, theta, normalize=False)
    pub.publish(v)

    # And plot if asked
    if show_plot:
        if abs(cx - x) > 10:
            cx = x
        if abs(cy - y) > 10:
            cy = y
        # X, Y = np.mgrid[cx - 50:cx + 50:40j, cy - 50:cy + 50:40j]
        X, Y = np.mgrid[-120:120:40j, -120:120:40j]
        U, V = behavior_manager.champ_total.get_field(
            X, Y, wind, theta, normalize=False)
        plt.cla()
        plt.quiver(X, Y, U, V, scale=100)
        plt.quiver(x, y, v.x, v.y, color='red')
        sb.x, sb.y, sb.theta, sb.X[2] = x, y, cap, cap
        sb.draw()
        sb.drawWind(awind, psi, coeff=10)
        ctr += 1
        plt.plot(trajx, trajy, 'r')
        if ctr % 5 == 0:
            trajx.append(sb.x)
            trajy.append(sb.y)
        plt.axis('equal')
        # draw_tank([x, y, cap])
        plt.draw()

    rospy.loginfo('-----------------------------------------------')

    rospy.loginfo('Published total cmd: {}, {}'.format(v.x, v.y))
    rospy.loginfo('Theta: {}, Wind: {}, Psi: {}'.format(theta, wind, psi))
    rate.sleep()
