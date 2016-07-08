#!/usr/bin/env python

import rospy
import numpy as np
from math import cos
from geometry_msgs.msg import Twist, Vector3, TwistStamped
from std_msgs.msg import Float32MultiArray


def update_cap(msg):
    """
    Met a jour le cap du bateau en recuperant son cap a partir
    du topic /gps/local_pose
    """
    global cap

    # print msg.pose.orientation, type(msg.pose.orientation)
    cap = msg.data[0]


def update_speed(msg):
    global vitesse_boat
    vitesse_boat = msg.twist.linear.x


def update_cible(msg):
    """
    Met a jour la vitesse reelle et le cap cible a partir
    du topic /robot/vecteur_cible
    """
    global cap_cible, V_vect, V_reel, V0

    cap_cible = np.arctan2(msg.y, msg.x)    # cap desire
    V_vect = np.sqrt(msg.x**2 + msg.y**2)   # vitesse vecteur

    # Vitesse reelle =  mise a l'echelle de V_vect
    V_reel = V0 + V_vect * (vHigh - V0)


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to', default
        return default


# Ros initialisation and rate
rospy.init_node('regulateur')
rate = rospy.Rate(5)


# Subscribes to gps position to calculate the heading
# S'abonne aux positions GPS pour pour recupere le cap
sub_cap = rospy.Subscriber("boat/compas", Float32MultiArray, update_cap)
sub_speed = rospy.Subscriber("boat/gps/velocity", TwistStamped, update_speed)

# Subscribes to the publisher to get the desired heading
# Recupe le cap desire
sub_consigne = rospy.Subscriber("robot/vecteur_cible", Vector3, update_cible)

# Publish the commands
# Publis les commandes
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


# Variables
cap_cible = 0
V_vect = 0
V_reel = 0
cap = 0
error = 0

# Parametres
V0 = fetch_param('~V0', 6000)
thetadot0 = fetch_param('~V0', 6000)
V_lim_reverse = fetch_param('~V_lim_reverse', 1.5)
vHigh = fetch_param('~vHigh', 8000)  # 8000 max
vLow = fetch_param('~vLow', 7000)  # 6000 = vitesse nulle
Kp = fetch_param('~Kp', 1500) * (2. / np.pi)
Kd = fetch_param('~Kd', 300) * (2. / np.pi)
reverse_motor = fetch_param('~reverse_motor', 1)

while not rospy.is_shutdown():
    cmd = Twist()

    # Regulation
    error_prev = error
    error = cap_cible - cap
    diff_error = (error - error_prev) / 0.2
    cmd.angular.z = thetadot0 + Kp * \
        np.arctan(np.tan((error / 2.))) + Kd * \
        np.arctan(np.tan((diff_error / 2.)))
    cmd.angular.z = reverse_motor * cmd.angular.z

    # bateau dans le sens du champ
    if cos(error) >= 0:
        # on bride
        if vLow > V_reel:
            cmd.linear.x = vLow
        elif V_reel > vHigh:
            cmd.linear.x = vHigh
        else:
            cmd.linear.x = V_reel
        print 'dans le sens du champ a', V_reel, ', ',

    # bateau dans le sens oppose au champ
    else:
        print 'contre le champ,  ',
        if V_vect >= V_lim_reverse:
            cmd.linear.x = -vHigh
            print 'marche arriere a -Vhigh, ',
        else:
            cmd.linear.x = vLow * 5
            print 'marche avant a Vlow, ',

    # Pour des champs tres faibles
    # (le champ n'est jamais nul lorsqu'un profil est une gaussienne)
    if V_vect <= 0.05:
        if V_vect == 0.0:
            print 'champ nul',
        else:
            print 'mais trop faible, ',
        cmd.linear.x = V0
        cmd.angular.z = thetadot0

    print
    # Publication de la commande
    cmd_pub.publish(cmd)
    rate.sleep()
