#!/usr/bin/env  python

# aller chercher la balle en suivi de ligne


import rospy
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PoseStamped


def ball_line(msg):
    poseBoatX = msg.local_pose.position.x
    poseBoatY = msg.local_pose.position.y
    poseBallX = msg.pose.position.x
    poseBallY = msg.pose.position.y

    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='001', f_type='line',
                        xa=poseBoatX, ya=poseBoatY,
                        xb=poseBallX, yb=poseBallY,
                        K=3, R=10)
    confirmation = behavior_sender(info, 'update')
    print confirmation

    if((poseBallX - 3 <= poseBoatX <= poseBallX + 3) & (poseBallY - 3 <= poseBoatY <= poseBallY + 3)):
        info2 = BehaviorInfo(behavior_id='001', f_type='waypoint',
                             xa=poseBallX, ya=poseBallY,
                             K=1,
                             security='HIGH', effect_range=10)
        confirmation2 = behavior_sender(info2, 'update')
        print confirmation2

rospy.init_node('search_ball')

rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# S'abonne aux positions GPS pour pour recuperer la position du bateau et
# de la balle
sub_pose_boat = rospy.Subscriber("gps/local_pose", PoseStamped, ball_line)
sub_pose_ball = rospy.Subscriber("ball/pose", PoseStamped, ball_line)

rospy.spin()
