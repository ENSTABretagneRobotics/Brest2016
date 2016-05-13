#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info = BehaviorInfo(behavior_id='001', type='ligne',
                    xa=39, ya=29, xb=35, yb=128, s=-1, r=100)

# Confirmation de la reception du behavior
confirmation = behavior_sender(info)
print confirmation
