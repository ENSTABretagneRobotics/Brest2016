#!/usr/bin/env python

import tf
import rospy
import numpy as np
from math import cos, sin, degrees
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from dynamic_reconfigure.server import Server
from process.cfg import dyn_pidConfig


def update_cap(msg):
    """
    Met a jour le cap du bateau en recuperant son cap a partir
    du topic /gps/local_pose
    """
    global cap

    # print msg.pose.orientation, type(msg.pose.orientation)
    cap = tf.transformations.euler_from_quaternion(
        [msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w])[2]


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


def callback(config, level):
    global Kp, Kd
    rospy.loginfo("""Reconfigure Request: {Kp}, {Kd}""".format(**config))
    Kp, Kd = config['Kp'], config['Kd']
    return config

# Ros initialisation and rate
rospy.init_node('regulateur')
rate = rospy.Rate(5)

# Server for dynamic reconfigure
srv = Server(dyn_pidConfig, callback)

# Subscribes to gps position to calculate the heading
# S'abonne aux positions GPS pour pour recupere le cap
sub_cap = rospy.Subscriber("gps/local_pose", PoseStamped, update_cap)

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
thetadot0 = fetch_param('~thetadot0', 6000)
V_lim_reverse = fetch_param('~V_lim_reverse', 1.5)
vHigh = fetch_param('~vHigh', 8000)  # 8000 max
vLow = fetch_param('~vLow', 7000)  # 6000 = vitesse nulle
Kp = fetch_param('~Kp', 1500) * (2. / np.pi)
Kd = fetch_param('~Kd', 0) * (2. / np.pi)
reverse_motor = fetch_param('~reverse_motor', 1)

# ###################################################################
#
# WHILE LOOP
#
# ###################################################################
while not rospy.is_shutdown():
    cmd = Twist()
    logmsg = ''

    # ###############################################################
    # Pour des champs tres faibles
    # (le champ n'est jamais nul lorsqu'un profil est une gaussienne)
    # ###############################################################
    if V_vect <= 0.05:
        if V_vect == 0.0:
            logmsg = 'champ nul'
        else:
            logmsg = 'champ quasi-nul'
        cmd.linear.x = V0
        cmd.angular.z = thetadot0

        # logs
        logform = '{}| vitesse: {:>8}% | angular: {:>8}%'
        logmsg = logform.format(logmsg, 0, 0)

        # pas la peine de reguler le cap pour des commandes inexistantes
        cap_cible = cap

    # ###############################################################
    # Si un champ existe a la position du bateau
    # ###############################################################
    else:
        # Erreur
        error_prev = error
        error = cap_cible - cap
        diff_error = error - error_prev
        # Inversion moteur
        rm = reverse_motor

        # Regulation cap
        cmd.angular.z = thetadot0 - rm * Kp * \
            np.arctan(np.tan((error / 2.))) - rm * Kd * \
            np.arctan(np.tan((diff_error / 2.)))

        # -----------------------------------------------------------
        # bateau dans le sens du champ
        # -----------------------------------------------------------
        if cos(error) >= 0:
            log_sens = 'face'
            log_turn = 'gauche' if sin(error) > 0 else 'droite'

            # on bride
            if vLow > V_reel:
                cmd.linear.x = vLow
            elif V_reel > vHigh:
                cmd.linear.x = vHigh
            else:
                cmd.linear.x = V_reel

        # -----------------------------------------------------------
        # bateau CONTRE le champ
        # -----------------------------------------------------------
        else:
            log_sens = 'contre'

            cmd.linear.x = vLow
            # il faut tourner a gauche
            if sin(error) > 0:
                cmd.angular.z = 8000
                log_turn = 'gauche'
            # il faut tourner a droite
            else:
                cmd.angular.z = 4000
                log_turn = 'droite'

        log_speed = (V_reel - V0) / 2000. * 100
        log_speedR = (cmd.linear.x - V0) / 2000. * 100
        log_rot = abs(cmd.angular.z - thetadot0) / 2000. * 100
        # logs
        logform = '{:<5.2f} {:<5}| tourne {:.2f}%  a {}| vitesse: {}% --> {}%'
        logmsg = logform.format(degrees(error), log_sens, log_rot,
                                log_turn, log_speed, log_speedR)

    rospy.loginfo(logmsg)
    # Publication de la commande
    cmd_pub.publish(cmd)
    rate.sleep()
