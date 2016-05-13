#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info3 = BehaviorInfo(behavior_id='003', type='patrol_circle',
                     xa=36, ya=80, xb=0, yb=0, s=1, r=5)

# Confirmation de la reception du behavior
confirmation = behavior_sender(info3)
print confirmation
