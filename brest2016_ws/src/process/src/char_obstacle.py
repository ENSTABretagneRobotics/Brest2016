#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PoseStamped

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)


def pose_obst(msg):
    poseObstX = msg.pose.position.x
    poseObstY = msg.pose.position.y
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='000', type='obst_point',
                        xa=poseObstX, ya=poseObstY, s=-1, r=30)
    confirmation = behavior_sender(info, 'update')
    print confirmation
rospy.init_node('char_obstacle')

# S'abonne aux positions GPS pour pour recupere la position de l'obstacle
sub_poseObst = rospy.Subscriber("obstacle/pose", PoseStamped, pose_obst)

rospy.spin()
