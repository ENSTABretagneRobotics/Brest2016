#!/usr/bin/env  python

# aller chercher la balle en suivi de ligne


import rospy
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PoseStamped
# from math import atan


def update_ball_pos(msg):
    global alpha, distance, ballX, ballY, ball_lost, ball_lost_time
    if msg.pose.position.x == -1 and msg.pose.position.y == -1:
        if not ball_lost:
            ball_lost = True
            ball_lost_time = rospy.Time.now()
            print 'ball just lost'

    else:
        if ball_lost:
            print 'ball just found'
        ball_lost = False
        ballX = msg.pose.position.x
        ballY = msg.pose.position.y
        # on calcul l'angle et la distance de la balle dans le repere du bateau
        # alpha = atan(ballY / ballX)
        # distance = (ballX**2 + ballY**2)**0.5


def define_behavior(msg):
    global ballX, ballY
    poseBoatX = msg.pose.position.x
    poseBoatY = msg.pose.position.y

    # Behavior a envoyer
    # Si on est proche de la balle on tourne autour
    # if((ballX - 3 <= poseBoatX <= ballX + 3) and (ballY - 3 <= poseBoatY <=
    # ballY + 3)):
    if ball_lost and (rospy.Time.now() - ball_lost_time).to_sec() > 30:
        info = BehaviorInfo(behavior_id='1001', f_type='patrol_circle',
                            xa=poseBoatX, ya=poseBoatY,
                            xb=0, yb=0,
                            K=3, R=10)
        print 'ball lost a long time ago, going back to patrol mode'
    else:
        info = BehaviorInfo(behavior_id='1001', f_type='waypoint',
                            xa=poseBoatX + ballX, ya=poseBoatY + ballY,
                            K=1,
                            security='HIGH', effect_range=10)
        print 'ball found, going there'
    confirmation = behavior_sender(info, 'update')
    print confirmation


rospy.init_node('go_to_ball')

rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy(
    'behavior_manager', behavior)

# S'abonne aux positions GPS pour pour recuperer la position du bateau et
# de la balle
sub_pose_boat = rospy.Subscriber(
    "gps/local_pose", PoseStamped, define_behavior)
sub_pose_ball = rospy.Subscriber("ball/pose", PoseStamped, update_ball_pos)

# Variables
alpha = 0
ball_lost = True
ball_lost_time = rospy.Time.now() - rospy.Duration(45)
# distance = -1
# ballX, ballY = -1, -1

rospy.spin()
