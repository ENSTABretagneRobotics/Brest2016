#!/usr/bin/env  python
import rospy
from process.srv import behavior, behaviorResponse
from geometry_msgs.msg import Vector3, PoseStamped
from Behavior import Behavior, BehaviorManager
import matplotlib.pyplot as plt
import numpy as np


def add_behavior(request):
    global listB
    print request.info.type
    behavior_manager.add_behavior(Behavior(request.info))
    print 'nombres de behavior recu:', len(behavior_manager.behavior_list)
    return behaviorResponse('oui')


def update_pos(msg):
    global x, y
    x = msg.pose.position.x
    y = msg.pose.position.y


# Variables
listB = []
x, y = 0, 0
behavior_manager = BehaviorManager()

# Initialisation du noeud
rospy.init_node('service_server')
# Initialisation du Serveur
service = rospy.Service('behavior_manager', behavior, add_behavior)
# Subscriber et publisher
pos_sub = rospy.Subscriber('gps/local_pose', PoseStamped, update_pos)
pub = rospy.Publisher('field_cmd', Vector3, queue_size=1)

rate = rospy.Rate(10)

# Affichage du champ de vecteur
plt.ion()
X, Y = np.mgrid[-200:200:40j, -200:200:40j]

while not rospy.is_shutdown():
    v = Vector3()
    v.x, v.y = behavior_manager.champ_total.cmd_point(x, y)
    print x, y, v.x, v.y
    U, V = behavior_manager.champ_total.cmd_point(X, Y)
    plt.cla()
    plt.quiver(X, Y, U, V)
    plt.quiver(x, y, v.x, v.y, color='red')
    plt.draw()
    pub.publish(v)
    rate.sleep()
