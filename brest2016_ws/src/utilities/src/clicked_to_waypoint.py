#!/usr/bin/env  python

# traduit un clic sous rviz en waypoint


import rospy
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PointStamped


def clicked_callback(msg):
    poseWayX = msg.point.x
    poseWayY = msg.point.y
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='001', f_type='waypoint',
                        xa=poseWayX, ya=poseWayY,
                        xb=0, yb=0,
                        K=1, R=3,
                        security='LOW', slowing_R=1,
                        slowing_K=1,
                        effect_range=4)
    confirmation = behavior_sender(info, 'update')
    print poseWayX, poseWayY, confirmation

rospy.init_node('behavior_client')

rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Subscriber to cliqued point
clicked_sub = rospy.Subscriber('clicked_point', PointStamped, clicked_callback)

rospy.spin()
