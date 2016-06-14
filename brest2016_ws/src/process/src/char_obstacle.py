#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PoseStamped


def pose_obst(msg):
    poseObstX = msg.pose.position.x
    poseObstY = msg.pose.position.y
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='001', f_type='obst_point',
                        xa=poseObstX, ya=poseObstY,
                        xb=0, yb=0,
                        K=4, R=3,
                        security='LOW', slowing_R=1,
                        slowing_K=1,
                        effect_range=4)
    confirmation = behavior_sender(info, 'update')
    print confirmation

rospy.init_node('char_obstacle')

rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# S'abonne aux positions GPS pour pour recupere la position de l'obstacle
sub_poseObst = rospy.Subscriber("obstacle/pose", PoseStamped, pose_obst)

rospy.spin()
