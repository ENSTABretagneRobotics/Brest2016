#!/usr/bin/env  python


# envoie un behavior au serveur


import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info = BehaviorInfo(behavior_id='001', f_type='obst_point',
                    xa=0, ya=0,
                    xb=0, yb=0,
                    K=4, R=3,
                    security='LOW', slowing_R=1,
                    slowing_K=1,
                    effect_range=4)
confirmation = behavior_sender(info, 'update')
