#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info1 = BehaviorInfo()
info1.behavior_id = '001'
info1.type = 'waypoint'
info1.xa = -5
info1.ya = -5
info1.s = -1

# Confirmation de la reception du behavior
confirmation = behavior_sender(info1)
print confirmation
