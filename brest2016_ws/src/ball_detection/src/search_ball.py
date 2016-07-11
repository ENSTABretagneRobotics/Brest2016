#!/usr/bin/env  python

# creer un champ de type patrouille circulaire
# pour chercher la position de la balle


import rospy
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PoseStamped


def pose_boat(msg):
    poseBoatX = msg.pose.position.x
    poseBoatY = msg.pose.position.y
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='1001', f_type='patrol_circle',
                        xa=poseBoatX, ya=poseBoatY,
                        xb=0, yb=0,
                        K=3, R=10)
    confirmation = behavior_sender(info, 'update')
    print confirmation

rospy.init_node('search_ball')

rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# S'abonne aux positions GPS pour pour recuperer la position du bateau
sub_pose_boat = rospy.Subscriber("gps/local_pose", PoseStamped, pose_boat)
